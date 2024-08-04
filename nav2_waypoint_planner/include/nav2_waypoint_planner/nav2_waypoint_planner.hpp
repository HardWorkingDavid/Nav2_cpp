#pragma once

#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_waypoint_planner {
class WaypointPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node {
public:
    WaypointPlanner() : Node("waypoint_node") {
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 100,
            std::bind(&WaypointPlanner::waypointCallback, this, std::placeholders::_1));
        costmap_ros_ = NULL;
        initialized_ = false;
        clear_waypoints_ = false;
    }

    ~WaypointPlanner() = default;

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

    bool makePlan(const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      std::vector<geometry_msgs::msg::PoseStamped>& plan);

    void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr waypoint) {
        if (clear_waypoints_)
        {
            waypoints_.clear();
            clear_waypoints_ = false;
        }

        // add waypoint to the waypoint vector
        geometry_msgs::msg::PoseStamped pose;
        waypoints_.push_back(pose);
        waypoints_.back().header = waypoint->header;
        waypoints_.back().pose.position = waypoint->point;
        waypoints_.back().pose.orientation.x = 0.0;
        waypoints_.back().pose.orientation.y = 0.0;
        waypoints_.back().pose.orientation.z = 0.0;
        waypoints_.back().pose.orientation.w = 1.0;

        // create and publish markers
        createAndPublishMarkersFromPath(waypoints_);


        if (waypoints_.size() < 2)
            return;

        geometry_msgs::msg::Pose *p1 = &(waypoints_.end()-2)->pose;
        geometry_msgs::msg::Pose *p2 = &(waypoints_.end()-1)->pose;

        // calculate orientation of waypoints
        double yaw = atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
        //p1->orientation = tf::createQuaternionMsgFromYaw(yaw);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw); // 设置四元数的欧拉角为 (roll, pitch, yaw)
        p1->orientation = tf2::toMsg(quaternion);

        // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
        double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);
        // 先把path_用在ros界面上点击的waypoints插进去的 再把path_的最后一个点当成goal点
        if (dist < epsilon_)
        {
            p2->orientation = p1->orientation;
            path_.header = waypoint->header;
            path_.poses.clear();
            path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
            
            goal_pub_->publish(waypoints_.back());
            clear_waypoints_ = true;
            std::cout << "Published goal pose" << std::endl;
        }
    }

    //void externalPathCallback(const nav_msgs::msg::Path::SharedPtr plan);

    void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::msg::PoseStamped>& path);

    void interpolatePath(nav_msgs::msg::Path& path);

private:
    bool initialized_;  //!< flag indicating the planner has been initialized
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
    //rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr external_path_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

    // configuration parameters
    double epsilon_;  //!< distance threshold between two waypoints that signifies the last waypoint
    int waypoints_per_meter_;  //!< number of waypoints per meter of generated path used for interpolation

    // containers
    //std::vector<geometry_msgs::msg::PoseStamped> waypoints_;  //!< container for the manually inserted waypoints
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    nav_msgs::msg::Path path_;  //!< container for the generated interpolated path

    //flags
    bool clear_waypoints_; 

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::string global_frame_, name_;
    rclcpp::Clock::SharedPtr clock_;
};


} // namespace nav2_waypoint_planner