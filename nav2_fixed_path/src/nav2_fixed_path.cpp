#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <fstream>

#include "nav2_fixed_path/nav2_fixed_path.hpp"

namespace nav2_fixed_path {

void FixedPath::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
}

void FixedPath::cleanup() {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type fixed_path_planner",
        name_.c_str());
}

void FixedPath::activate() {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type fixed_path_planner",
        name_.c_str());
}

void FixedPath::deactivate() {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type fixed_path_planner",
        name_.c_str());
}

nav_msgs::msg::Path FixedPath::createPlan(const geometry_msgs::msg::PoseStamped & start,
                                          const geometry_msgs::msg::PoseStamped & goal) {
    nav_msgs::msg::Path global_path;

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    std::ifstream filename("/home/bigdavid/Nav2/src/record_path/path_data/path_data.csv", std::ios::in);
    if (!filename.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return global_path;
    }

    std::string line;
    bool header_skipped = false;
    while (std::getline(filename, line)) {
        if (!header_skipped) {
            header_skipped = true;
            continue;
        }
        geometry_msgs::msg::PoseStamped pose;
        std::istringstream ss(line);
        ss >> pose.pose.position.x 
           >> pose.pose.position.y 
           >> pose.pose.position.z 
           >> pose.pose.orientation.x 
           >> pose.pose.orientation.y 
           >> pose.pose.orientation.z 
           >> pose.pose.orientation.w;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        global_path.poses.push_back(pose);
    }

    filename.close();
    path_publisher_->publish(global_path);

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
    return global_path;
}

} // namespace nav2_fixed_path

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_fixed_path::FixedPath, nav2_core::GlobalPlanner)