/**
 * *********************************************************
 *
 * @file: local_planner.h
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
#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/controller.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "math_helper.h"

namespace local_planner
{
class LocalPlanner
{
public:
  /**
   * @brief Construct a new Local Planner object
   */
  LocalPlanner();

  /**
   * @brief Destroy the Local Planner object
   */
  ~LocalPlanner();

  /**
   * @brief Set or reset frame name
   * @param frame_name
   */
  void setBaseFrame(std::string base_frame);
  void setMapFrame(std::string map_frame);

  /**
   * @brief Regularize angle to [-pi, pi]
   * @param angle the angle (rad) to regularize
   * @return reg_angle the regulated angle
   */
  double regularizeAngle(double angle);

  /**
   * @brief Get the Euler Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  roll, pitch and yaw in XYZ order
   */
  Eigen::Vector3d getEulerAngles(geometry_msgs::msg::PoseStamped& ps);

  /**
   * @brief Whether to reach the target pose through rotation operation
   * @param cur   current pose of robot
   * @param goal  goal pose of robot
   * @return true if robot should perform rotation
   */
  bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& cur, const geometry_msgs::msg::PoseStamped& goal);

  /**
   * @brief Whether to correct the tracking path with rotation operation
   * @param angle_to_path  the angle deviation
   * @return true if robot should perform rotation
   */
  bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

  /**
   * @brief linear velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return v            regulated linear velocity
   */
  double linearRegularization(double v, double v_d);

  /**
   * @brief angular velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param w_d           desired angular velocity
   * @return  w           regulated angular velocity
   */
  double angularRegularization(double w, double w_d);

  /**
   * @brief Tranform from in_pose to out_pose with out frame using tf
   */
  void transformPose(const std::string out_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose) const;

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   * @return true if successfull, else false
   */
  bool worldToMap(double wx, double wy, int& mx, int& my);

  /**
   * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
   * @param robot_pose_global the robot's pose  [global]
   * @return pruned path
   */
  std::vector<geometry_msgs::msg::PoseStamped> prune(const geometry_msgs::msg::PoseStamped robot_pose_global);

  /**
   * @brief Calculate the look-ahead distance with current speed dynamically
   * @param vt the current speed
   * @return L the look-ahead distance
   */
  double getLookAheadDistance(double vt);

  /**
   * @brief find the point on the path that is exactly the lookahead distance away from the robot
   * @param lookahead_dist    the lookahead distance
   * @param robot_pose_global the robot's pose  [global]
   * @param prune_plan        the pruned plan
   * @param pt                the lookahead point
   * @param theta             the angle on traj
   * @param kappa             the curvature on traj
   */
  void getLookAheadPoint(double lookahead_dist, geometry_msgs::msg::PoseStamped robot_pose_global,
                         const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan,
                         geometry_msgs::msg::PointStamped& pt, double& theta, double& kappa);

protected:
  rclcpp::Logger logger_{ rclcpp::get_logger("LocalPlanner") };

  std::shared_ptr<tf2_ros::Buffer> tf_;

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  // if the distance is less than the tolerance value, it is considered to have reached the target position
  double goal_dist_tol_;

  // if the angle deviation is greater than this threshold, perform rotation first
  double rotate_tol_;

  // frame name of base link, map and odometry
  std::string base_frame_, map_frame_, odom_frame_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  nav_msgs::msg::Path global_plan_;

  double lookahead_time_;      // lookahead time gain
  double min_lookahead_dist_;  // minimum lookahead distance
  double max_lookahead_dist_;  // maximum lookahead distance
};
}  // namespace local_planner

#endif