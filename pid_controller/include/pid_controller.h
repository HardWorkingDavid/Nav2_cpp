
#ifndef NAV2_PID_CONTROLLER_H_
#define NAV2_PID_CONTROLLER_H_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "local_planner.h"

namespace pid_controller
{

/**
 * @class pid_controller::PIDController
 * @brief PIDController plugin
 */
class PIDController : public nav2_core::Controller, local_planner::LocalPlanner
{
public:
  /**
   * @brief Constructor for pid_controller::PIDController
   */
  PIDController() = default;

  /**
   * @brief Destrructor for pid_controller::PIDController
   */
  ~PIDController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  // 需要一个新的速度命令以便机器人跟随全局路径时调用该方法
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* /*goal_checker*/) override;

  /**
   * @brief Sets the global plan
   * @param path The global plan
   */
  // 当全局路径更新时，调用该方法，此方法执行转换、处理全局路径并存储的操作
  void setPlan(const nav_msgs::msg::Path& path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  // 当需要限制机器人的最大线速度时调用该方法，速度限制可以用绝对值或最大机器人速度的百分比表示
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
  /**
   * @brief Execute PID control process (no model pid)
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered input
   * @return u  control vector
   */
  Eigen::Vector2d _pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;

  rclcpp::Logger logger_{ rclcpp::get_logger("PIDController") };
  rclcpp::Clock::SharedPtr clock_;

  // pid controller params
  double k_v_p_, k_v_i_, k_v_d_;
  double k_w_p_, k_w_i_, k_w_d_;
  double k_theta_;
  double k_, l_;

  double e_v_, e_w_;
  double i_v_, i_w_;

  double d_t_;

  // goal parameters
  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> target_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> current_pose_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> collision_checker_;

  // Dynamic parameters handler
  std::mutex mutex_;
};

}  // namespace pid_controller

#endif
