#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include "nav2_core/global_planner.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "voronoi.hpp"

using namespace std::chrono_literals;

// 定义 PI 常量
const double PI = 3.14159265358979323846;

namespace nav2_voronoi_planner
{
class VoronoiPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node
{
public:
    VoronoiPlanner() : Node("voronoi_node") { 
        /*start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&VoronoiPlanner::SetStart, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&VoronoiPlanner::SetGoal, this, std::placeholders::_1));*/
        // 订阅全局路径消息
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_path", 10, std::bind(&VoronoiPlanner::global_path_callback, this, std::placeholders::_1));

        // 发布处理后的路径消息
        processed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("processed_path", 10);
    }
    ~VoronoiPlanner() {
        RCLCPP_INFO(
            logger_, "Destroying plugin %s of type VoronoiPlanner",
            name_.c_str());
    }

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

protected:
    // 归一化角度到 (-π, π] 范围内
    double normalize_angle(double angle) {
        while (angle > PI) {
            angle -= 2 * PI;
        }
        while (angle <= -PI) {
            angle += 2 * PI;
        }
        return angle;
    }

    // 计算两点之间的欧氏距离
    double distance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2) {
        return std::sqrt(std::pow(p2.point.x - p1.point.x, 2) + std::pow(p2.point.y - p1.point.y, 2));
    }

    // 在 insert_points 函数中确保使用 PoseStamped 类型
    nav_msgs::msg::Path insert_points(const nav_msgs::msg::Path& path_msg, double distance_threshold) {
        nav_msgs::msg::Path new_path;
        new_path.header = path_msg.header;

        for (size_t i = 1; i < path_msg.poses.size(); ++i) {
            const auto& prev_pose = path_msg.poses[i - 1];
            const auto& curr_pose = path_msg.poses[i];

            // 从 PoseStamped 中提取位置信息，构造 PointStamped
            geometry_msgs::msg::PointStamped prev_point;
            prev_point.point.x = prev_pose.pose.position.x;
            prev_point.point.y = prev_pose.pose.position.y;

            geometry_msgs::msg::PointStamped curr_point;
            curr_point.point.x = curr_pose.pose.position.x;
            curr_point.point.y = curr_pose.pose.position.y;

            double dist = distance(prev_point, curr_point);

            new_path.poses.push_back(prev_pose);

            if (dist > distance_threshold) {
                int num_insertions = std::floor(dist / distance_threshold);
                for (int j = 1; j <= num_insertions; ++j) {
                    double ratio = static_cast<double>(j) / (num_insertions + 1);
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.pose.position.x = prev_pose.pose.position.x + (curr_pose.pose.position.x - prev_pose.pose.position.x) * ratio;
                    pose_stamped.pose.position.y = prev_pose.pose.position.y + (curr_pose.pose.position.y - prev_pose.pose.position.y) * ratio;
                    new_path.poses.push_back(pose_stamped);
                }
            }
        }

        // new_path.poses.push_back(path_msg.poses.back());
        // Remove the last pose if new_path.poses is not empty
        if (!new_path.poses.empty()) {
            new_path.poses.pop_back();
        }

        return new_path;
    }

    // 调整路径点，平滑路径
    nav_msgs::msg::Path adjust_points(const nav_msgs::msg::Path& path_msg, double angle_threshold) {
        nav_msgs::msg::Path adjusted_path;
        adjusted_path.header = path_msg.header;
        adjusted_path.poses.push_back(path_msg.poses.front());

        for (size_t i = 2; i < path_msg.poses.size(); ++i) {
            const auto& prev_pose = path_msg.poses[i - 1];
            const auto& curr_pose = path_msg.poses[i];
            const auto& next_pose = path_msg.poses[i + 1];

            double angle1 = std::atan2(curr_pose.pose.position.y - prev_pose.pose.position.y, curr_pose.pose.position.x - prev_pose.pose.position.x);
            double angle2 = std::atan2(next_pose.pose.position.y - curr_pose.pose.position.y, next_pose.pose.position.x - curr_pose.pose.position.x);
            double angle_diff = normalize_angle(angle2 - angle1);

            if (std::abs(angle_diff) > angle_threshold) {
                geometry_msgs::msg::PoseStamped mid_pose;
                mid_pose.pose.position.x = (curr_pose.pose.position.x + next_pose.pose.position.x) / 2;
                mid_pose.pose.position.y = (curr_pose.pose.position.y + next_pose.pose.position.y) / 2;
                adjusted_path.poses.push_back(mid_pose);
            } else {
                adjusted_path.poses.push_back(curr_pose);
            }
        }

        adjusted_path.poses.push_back(path_msg.poses.back());

        return adjusted_path;
    }

    

    // 全局路径回调函数
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // 进行路径处理
        nav_msgs::msg::Path processed_path = process_path(*msg);

        // 发布处理后的路径消息
        processed_path_pub_->publish(processed_path);
    }

    // 路径处理函数
    nav_msgs::msg::Path process_path(const nav_msgs::msg::Path& input_path) {
        nav_msgs::msg::Path processed_path = input_path;

        // 示例处理：插入点、调整点、优化路径
        processed_path = insert_points(processed_path, 1.0);
        processed_path = adjust_points(processed_path, PI / 6);

        return processed_path;
    }



    bool makePlan(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  nav_msgs::msg::Path& plan);
    bool UpdateCostmap(nav2_costmap_2d::Costmap2DROS* costmap_ros);
    static void GetStartAndEndConfigurations(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal, double resolution,
        double origin_x, double origin_y, int* start_x, int* start_y, int* end_x,
        int* end_y);
    std::vector<std::vector<VoronoiData>> GetVoronoiDiagram(unsigned int size_x,
                                                            unsigned int size_y,
                                                            double resolution);

    static void PopulateVoronoiPath(
        const std::vector<std::pair<int, int>>& searched_result,
        const std_msgs::msg::Header& header, double resolution, double origin_x,
        double origin_y, nav_msgs::msg::Path & plan);

    void PublishVoronoiGrid(const DynamicVoronoi& voronoi,
                            const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub);

    void PublishVoronoiPath(
        nav_msgs::msg::Path& plan,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub);
private:
    std::unique_ptr<Voronoi> voronoi_planner_ = nullptr;
    //const nav2_costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    nav2_costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    nav2_costmap_2d::Costmap2D * costmap_2d_ = nullptr;
    nav2_costmap_2d::LayeredCostmap* layered_costmap_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::string global_frame_, name_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("VoronoiPlanner")};
    bool initialized_ = false;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voronoi_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    /*rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    geometry_msgs::msg::PoseStamped start_;
    geometry_msgs::msg::PoseStamped goal_;
    bool start_received_ = false;
    bool goal_received_  = false;

    std::mutex start_mutex_;
    std::mutex goal_mutex_;*/
    // ROS 订阅器和发布器
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub_;

};

}
