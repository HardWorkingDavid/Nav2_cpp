/**
 * *********************************************************
 *
 * @file: local_planner.cpp
 * @brief: Contains the abstract local planner class
 * @author: Yang Haodong
 * @date: 2024-01-20
 * @version: 1.3
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>

#include "local_planner.h"

namespace local_planner
{
/**
 * @brief Construct a new Local Planner object
 */
LocalPlanner::LocalPlanner() : base_frame_("base_link"), map_frame_("map"), odom_frame_("odom"), costmap_ros_(nullptr)
{
}

/**
 * @brief Destroy the Local Planner object
 */
LocalPlanner::~LocalPlanner()
{
}

/**
 * @brief Set or reset frame name
 * @param frame_name
 */
void LocalPlanner::setBaseFrame(std::string base_frame)
{
  base_frame_ = base_frame;
}
void LocalPlanner::setMapFrame(std::string map_frame)
{
  map_frame_ = map_frame;
}

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 * @return reg_angle the regulated angle
 */
double LocalPlanner::regularizeAngle(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * @brief Get the Euler Angles from PoseStamped
 * @param ps PoseStamped to calculate
 * @return roll, pitch and yaw in XYZ order
 */
Eigen::Vector3d LocalPlanner::getEulerAngles(geometry_msgs::msg::PoseStamped& ps)
{
  tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double roll(0.0), pitch(0.0), yaw(0.0);
  m.getRPY(roll, pitch, yaw);

  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief Whether to reach the target pose through rotation operation
 * @param cur  current pose of robot
 * @param goal goal pose of robot
 * @return true if robot should perform rotation
 */
bool LocalPlanner::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& cur,
                                      const geometry_msgs::msg::PoseStamped& goal)
{
  std::pair<double, double> p1(cur.pose.position.x, cur.pose.position.y);
  std::pair<double, double> p2(goal.pose.position.x, goal.pose.position.y);

  return helper::dist(p1, p2) < goal_dist_tol_;
}

/**
 * @brief Whether to correct the tracking path with rotation operation
 * @param angle_to_path the angle deviation
 * @return true if robot should perform rotation
 */
bool LocalPlanner::shouldRotateToPath(double angle_to_path, double tolerance)
{
  return (tolerance && (angle_to_path > tolerance)) || (!tolerance && (angle_to_path > rotate_tol_));
}

/**
 * @brief linear velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param v_d           desired velocity magnitude
 * @return v            regulated linear velocity
 */
double LocalPlanner::linearRegularization(double v, double v_d)
{
  double v_inc = v_d - v;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param w_d           desired angular velocity
 * @return w            regulated angular velocity
 */
double LocalPlanner::angularRegularization(double w, double w_d)
{
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w_inc = w_d - w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  return w_cmd;
}

/**
 * @brief Tranform from in_pose to out_pose with out frame using tf
 */
void LocalPlanner::transformPose(const std::string out_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                 geometry_msgs::msg::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == out_frame)
    out_pose = in_pose;

  tf_->transform(in_pose, out_pose, out_frame);
  out_pose.header.frame_id = out_frame;
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool LocalPlanner::worldToMap(double wx, double wy, int& mx, int& my)
{
  unsigned int mx_u, my_u;
  bool flag = costmap_->worldToMap(wx, wy, mx_u, my_u);
  mx = static_cast<int>(mx_u);
  my = static_cast<int>(my_u);

  return flag;
}

/**
 * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
 * @param robot_pose_global the robot's pose  [global]
 * @return pruned path
 */
std::vector<geometry_msgs::msg::PoseStamped>
LocalPlanner::prune(const geometry_msgs::msg::PoseStamped robot_pose_global)
{
  auto calPoseDistance = [](const geometry_msgs::msg::PoseStamped& ps_1, const geometry_msgs::msg::PoseStamped& ps_2) {
    return helper::dist(ps_1, ps_2);
  };

  auto closest_pose_upper_bound =
      helper::firstIntegratedDistance(global_plan_.poses.begin(), global_plan_.poses.end(), calPoseDistance,
                                      costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);

  // find the closest pose on the path to the robot
  auto transform_begin = helper::getMinFuncVal(
      global_plan_.poses.begin(), closest_pose_upper_bound,
      [&](const geometry_msgs::msg::PoseStamped& ps) { return calPoseDistance(robot_pose_global, ps); });

  // Transform the near part of the global plan into the robot's frame of reference.
  std::vector<geometry_msgs::msg::PoseStamped> prune_path;
  for (auto it = transform_begin; it < global_plan_.poses.end(); it++)
    prune_path.push_back(*it);

  // path pruning: remove the portion of the global plan that already passed so don't process it on the next iteration
  global_plan_.poses.erase(std::begin(global_plan_.poses), transform_begin);

  return prune_path;
}

/**
 * @brief Calculate the look-ahead distance with current speed dynamically
 * @param vt the current speed
 * @return L the look-ahead distance
 */
double LocalPlanner::getLookAheadDistance(double vt)
{
  double lookahead_dist = fabs(vt) * lookahead_time_;
  return helper::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

/**
 * @brief find the point on the path that is exactly the lookahead distance away from the robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @param pt                the lookahead point
 * @param theta             the angle on traj
 * @param kappa             the curvature on traj
 */
void LocalPlanner::getLookAheadPoint(double lookahead_dist, geometry_msgs::msg::PoseStamped robot_pose_global,
                                     const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan,
                                     geometry_msgs::msg::PointStamped& pt, double& theta, double& kappa)
{
  double rx = robot_pose_global.pose.position.x;
  double ry = robot_pose_global.pose.position.y;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(prune_plan.begin(), prune_plan.end(), [&](const auto& ps) {
    return helper::dist(ps, robot_pose_global) >= lookahead_dist;
  });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == prune_plan.end())
  {
    goal_pose_it = std::prev(prune_plan.end());
    pt.point.x = goal_pose_it->pose.position.x;
    pt.point.y = goal_pose_it->pose.position.y;
    kappa = 0;
    theta = atan2(pt.point.y - ry, pt.point.x - rx);
  }
  else
  {
    // find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    double px, py;
    double gx = goal_pose_it->pose.position.x;
    double gy = goal_pose_it->pose.position.y;
    if (goal_pose_it == prune_plan.begin())
    {
      px = rx;
      py = ry;
    }
    else
    {
      auto prev_pose_it = std::prev(goal_pose_it);
      px = prev_pose_it->pose.position.x;
      py = prev_pose_it->pose.position.y;
    }

    // transform to the robot frame so that the circle centers at (0,0)
    std::pair<double, double> prev_p(px - rx, py - ry);
    std::pair<double, double> goal_p(gx - rx, gy - ry);
    std::vector<std::pair<double, double>> i_points = helper::circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    pt.point.x = i_points[0].first + rx;
    pt.point.y = i_points[0].second + ry;

    auto next_pose_it = std::next(goal_pose_it);
    if (next_pose_it != prune_plan.end())
    {
      double ax = px;
      double ay = py;
      double bx = gx;
      double by = gy;
      double cx = next_pose_it->pose.position.x;
      double cy = next_pose_it->pose.position.y;
      double a = std::hypot(bx - cx, by - cy);
      double b = std::hypot(cx - ax, cy - ay);
      double c = std::hypot(ax - bx, ay - by);

      double cosB = (a * a + c * c - b * b) / (2 * a * c);
      double sinB = std::sin(std::acos(cosB));

      double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
      kappa = std::copysign(2 * sinB / b, cross);
    }
    else
    {
      kappa = 0.0;
    }
    theta = atan2(gy - py, gx - px);
  }

  if (std::isnan(kappa))
    kappa = 0.0;
  pt.header.frame_id = goal_pose_it->header.frame_id;
  pt.header.stamp = goal_pose_it->header.stamp;
}

}  // namespace local_planner