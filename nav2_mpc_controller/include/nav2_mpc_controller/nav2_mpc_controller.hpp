/* File Info 
 * Author:      BigDavid 
 * CreateTime:  2024/8/1 23:49:05 
 * LastEditor:  BigDavid 
 * ModifyTime:  2024/8/1 23:49:08 
 * Description: mpc_controller头文件
*/ 
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <eigen3/Eigen/Dense>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "local_planner.h"


namespace nav2_mpc_controller {

class MpcController : public nav2_core::Controller, public local_planner::LocalPlanner
{
public:
    MpcController() = default;
    ~MpcController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
    void cleanup();
    void activate();
    void deactivate();
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker);

    void setPlan(const nav_msgs::msg::Path & path);
    void setSpeedLimit(const double & speed_limit, const bool & percentage);
private:
    Eigen::Vector2d mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r, Eigen::Vector2d du_p);
protected:
    bool initialized_ = false;     // initialized flag
    bool goal_reached_ = false;    // goal reached flag
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;

    rclcpp::Logger logger_{ rclcpp::get_logger("MPCController") };
    rclcpp::Clock::SharedPtr clock_;

    double d_t_;            // control time interval
    Eigen::Matrix3d Q_;     // state error matrix
    Eigen::Matrix2d R_;     // control error matrix
    int p_;                 // predicting time domain
    int m_;                 // control time domain
    Eigen::Vector2d du_p_;  // previous control error

    // goal parameters
    double goal_x_, goal_y_;
    Eigen::Vector3d goal_rpy_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> target_pt_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> current_pose_pub_;
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> collision_checker_;

    // Dynamic parameters handler
    std::mutex mutex_;
};



} // namespace nav2_mpc_controller