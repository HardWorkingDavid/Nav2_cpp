#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "/home/bigdavid/Nav2/src/navigation2/nav2_controller/include/nav2_controller/controller_server.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_nav");

    //auto node1 = std::make_shared<nav2_controller::ControllerServer>();

    YAML::Node config = YAML::LoadFile("/home/bigdavid/Nav2/src/multi_nav/config/mul_goal.yaml");
    const auto& goals = config["goals"];
    
    //node1->declare_parameter("FollowPath.use_collision_detection",false);

    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1000);
    for (const auto& goal : goals) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp.sec = rclcpp::Clock().now().seconds();
        goal_pose.header.stamp.nanosec = rclcpp::Clock().now().nanoseconds();
        goal_pose.pose.position.x = goal["x"].as<double>();
        goal_pose.pose.position.y = goal["y"].as<double>();
        std::cout << goal_pose.pose.position.x << ' ' << goal_pose.pose.position.y << std::endl;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, goal["yaw"].as<double>());
        tf2::Quaternion orientation;
        tf2::convert(quaternion, orientation);
        goal_pose.pose.orientation = tf2::toMsg(orientation);
    //    node1->set_parameter(rclcpp::Parameter("FollowPath.use_collision_detection", true));
        // 发布goal
        std::this_thread::sleep_for(std::chrono::seconds(3)); 
        goal_pub->publish(goal_pose);

        // 过20s之后再发下一个goal_pose
        std::this_thread::sleep_for(std::chrono::seconds(20)); 
    }

    return 0;
}