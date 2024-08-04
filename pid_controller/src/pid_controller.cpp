/**
 * *********************************************************
 *
 * @file: pid_planner.cpp
 * @brief: Contains the Proportional–Integral–Derivative (PID) controller local planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @date: 2024-01-20
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu, Wu Maojia.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <nav2_core/exceptions.hpp>

#include "pid_controller.h"

using nav2_util::declare_parameter_if_not_declared;

namespace pid_controller
{
void PIDController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node)
  {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // base
  declare_parameter_if_not_declared(node, plugin_name_ + ".goal_dist_tolerance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".map_frame", rclcpp::ParameterValue("map"));
  node->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tol_);
  node->get_parameter(plugin_name_ + ".rotate_tolerance", rotate_tol_);
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node->get_parameter(plugin_name_ + ".map_frame", map_frame_);

  // lookahead
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);

  // linear velocity
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_v", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_v_inc", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".max_v", max_v_);
  node->get_parameter(plugin_name_ + ".min_v", min_v_);
  node->get_parameter(plugin_name_ + ".max_v_inc", max_v_inc_);

  // angular velocity
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.57));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_w", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_w_inc", rclcpp::ParameterValue(1.57));
  node->get_parameter(plugin_name_ + ".max_w", max_w_);
  node->get_parameter(plugin_name_ + ".min_w", min_w_);
  node->get_parameter(plugin_name_ + ".max_w_inc", max_w_inc_);

  // PID parameters
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_v_p", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_v_i", rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_v_d", rclcpp::ParameterValue(1.10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_w_p", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_w_i", rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_w_d", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_theta", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".l", rclcpp::ParameterValue(0.2));
  node->get_parameter(plugin_name_ + ".k_v_p", k_v_p_);
  node->get_parameter(plugin_name_ + ".k_v_i", k_v_i_);
  node->get_parameter(plugin_name_ + ".k_v_d", k_v_d_);
  node->get_parameter(plugin_name_ + ".k_w_p", k_w_p_);
  node->get_parameter(plugin_name_ + ".k_w_i", k_w_i_);
  node->get_parameter(plugin_name_ + ".k_w_d", k_w_d_);
  node->get_parameter(plugin_name_ + ".k_theta", k_theta_);
  node->get_parameter(plugin_name_ + ".k", k_);
  node->get_parameter(plugin_name_ + ".l", l_);

  e_v_ = i_v_ = 0.0;
  e_w_ = i_w_ = 0.0;

  double controller_freqency;
  node->get_parameter("controller_frequency", controller_freqency);
  d_t_ = 1 / controller_freqency;

  target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  current_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

  RCLCPP_INFO(logger_, "PID planner initialized!");

  // initialize collision checker and set costmap
  collision_checker_ =
      std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void PIDController::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
              plugin_name_.c_str());
  target_pose_pub_.reset();
  current_pose_pub_.reset();
}

void PIDController::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
              plugin_name_.c_str());
  target_pose_pub_->on_activate();
  current_pose_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
}

void PIDController::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
              plugin_name_.c_str());
  target_pose_pub_->on_deactivate();
  current_pose_pub_->on_deactivate();
}

void PIDController::setPlan(const nav_msgs::msg::Path& path)
{
  RCLCPP_INFO(logger_, "Got new plan");
  global_plan_ = path;

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.poses.back().pose.position.x || goal_y_ != global_plan_.poses.back().pose.position.y)
  {
    goal_x_ = global_plan_.poses.back().pose.position.x;
    goal_y_ = global_plan_.poses.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.poses.back());

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return true if a valid trajectory was found, else false
 */
geometry_msgs::msg::TwistStamped PIDController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                        const geometry_msgs::msg::Twist& speed,
                                                                        nav2_core::GoalChecker* goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
  {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  }
  else
  {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // get robot position in map
  geometry_msgs::msg::PoseStamped current_ps_map, target_ps_map;
  transformPose(map_frame_, pose, current_ps_map);

  // prune the global plan
  std::vector<geometry_msgs::msg::PoseStamped> prune_plan = prune(current_ps_map);

  // calculate look-ahead distance
  double vt = std::hypot(speed.linear.x, speed.linear.y);
  double wt = speed.angular.z;
  double L = getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  geometry_msgs::msg::PointStamped lookahead_pt;
  double theta_d, theta_dir, theta_trj, kappa;
  getLookAheadPoint(L, current_ps_map, prune_plan, lookahead_pt, theta_trj, kappa);

  target_ps_map.pose.position.x = lookahead_pt.point.x;
  target_ps_map.pose.position.y = lookahead_pt.point.y;
  theta_dir = atan2((target_ps_map.pose.position.y - current_ps_map.pose.position.y),
                    (target_ps_map.pose.position.x - current_ps_map.pose.position.x));
  theta_d = regularizeAngle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_map.pose.orientation);

  // current angle
  double theta = tf2::getYaw(current_ps_map.pose.orientation);  // [-pi, pi]

  // position reached
  geometry_msgs::msg::TwistStamped cmd_vel;
  if (shouldRotateToGoal(current_ps_map, global_plan_.poses.back()))
  {
    double e_theta = regularizeAngle(goal_rpy_.z() - theta);

    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
    }
    // orientation not reached
    else
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = angularRegularization(speed.angular.z, e_theta / d_t_);
    }
  }
  // posistion not reached
  else
  {
    Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);    // current state
    Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);  // desired state
    Eigen::Vector2d u_r(vt, wt);                                                                 // refered input
    Eigen::Vector2d u = _pidControl(s, s_d, u_r);

    cmd_vel.twist.linear.x = linearRegularization(std::hypot(speed.linear.x, speed.linear.y), u[0]);
    cmd_vel.twist.angular.z = angularRegularization(speed.angular.z, u[1]);
  }

  // publish next target_ps_map pose
  target_ps_map.header.frame_id = "map";
  target_ps_map.header.stamp = current_ps_map.header.stamp;
  target_pose_pub_->publish(target_ps_map);

  // publish robot pose
  current_ps_map.header.frame_id = "map";
  current_ps_map.header.stamp = current_ps_map.header.stamp;
  current_pose_pub_->publish(current_ps_map);

  // populate and return message
  cmd_vel.header = pose.header;

  return cmd_vel;
}

void PIDController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  //   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
  //     // Restore default value
  //     desired_linear_vel_ = base_desired_linear_vel_;
  //   } else {
  //     if (percentage) {
  //       // Speed limit is expressed in % from maximum speed of robot
  //       desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
  //     } else {
  //       // Speed limit is expressed in absolute value
  //       desired_linear_vel_ = speed_limit;
  //     }
  //   }
}

/**
 * @brief Execute PID control process (with model)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PIDController::_pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e(s_d - s);
  Eigen::Vector2d sx_dot(k_ * e[0], k_ * e[1]);
  Eigen::Matrix2d R_inv;
  R_inv << cos(s[2]), sin(s[2]), -sin(s[2]) / l_, cos(s[2]) / l_;
  u = R_inv * sx_dot;

  return u;
}
}  // namespace pid_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pid_controller::PIDController, nav2_core::Controller)