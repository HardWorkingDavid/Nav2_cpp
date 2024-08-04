#include "nav2_waypoint_planner/nav2_waypoint_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

visualization_msgs::msg::MarkerArray markers;
visualization_msgs::msg::Marker marker;

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_waypoint_planner {

void WaypointPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    if (!initialized_) {
        // get the costmap
        costmap_ros_ = costmap_ros;
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".epsilon", rclcpp::ParameterValue(
            0.1));
        node_->get_parameter(name_ + ".epsilon", epsilon_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".waypoints_per_meter", rclcpp::ParameterValue(
            20));
        node_->get_parameter(name_ + ".waypoints_per_meter", waypoints_per_meter_);

        

        /*external_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "external_path", rclcpp::QoS(1),
            std::bind(&WaypointPlanner::externalPathCallback, this, std::placeholders::_1));*/
        
        // ROS 2 publishers
        waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pub_waypoints", 1);

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1); 

        plan_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/fixed_plan", 1); 

        initialized_ = true;

        std::cout << "Planner has been initialized" << std::endl;
    } else {
        std::cout << "This planner has already been initialized" << std::endl;
    }
}

void WaypointPlanner::cleanup()
{
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type WaypointPlanner",
        name_.c_str());
}

void WaypointPlanner::activate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type WaypointPlanner",
        name_.c_str());
}

void WaypointPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type WaypointPlanner",
    name_.c_str());
}

nav_msgs::msg::Path WaypointPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) {

    nav_msgs::msg::Path global_path;
    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
        RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
        return global_path;
    }

    global_path.poses.clear();

    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;

    if (!makePlan(start, goal, global_path.poses)) {
        std::cout << "failed to create plan" << std::endl;
    }
    return global_path;
}

bool WaypointPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                               const geometry_msgs::msg::PoseStamped& goal,
                               std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    (void)goal;
    path_.poses.insert(path_.poses.begin(), start);
    std::cout << "path_ 的长度: " << path_.poses.size() << std::endl;
    interpolatePath(path_);
    plan_pub_->publish(path_);
    plan = path_.poses;
    std::cout << "Published global plan" << std::endl;
    return true;
}



// 插值路径
void WaypointPlanner::interpolatePath(nav_msgs::msg::Path& path) {
    std::vector<geometry_msgs::msg::PoseStamped> temp_path;
    for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
    {
        // calculate distance between two consecutive waypoints
        double x1 = path.poses[i].pose.position.x;
        double y1 = path.poses[i].pose.position.y;
        double x2 = path.poses[i+1].pose.position.x;
        double y2 = path.poses[i+1].pose.position.y;
        double dist =  hypot(x1-x2, y1-y2);
        int num_wpts = std::floor(dist * waypoints_per_meter_);

        // 储存waypoints前一点
        temp_path.push_back(path.poses[i]);
        geometry_msgs::msg::PoseStamped p = path.poses[i];
        for (int j = 1; j < num_wpts - 2; j++)
        {
            p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
            p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
            temp_path.push_back(p);
        }
    }

    // update sequence of poses
    for (size_t i = 0; i < temp_path.size(); i++) {
        temp_path[i].header.stamp.sec = static_cast<int32_t>(i);
        temp_path[i].header.stamp.nanosec = 0;
    }
        
    //把终点存在temp_path中
    temp_path.push_back(path.poses.back());
    path.poses = temp_path;
}


// 外部路径回调
/*void WaypointPlanner::externalPathCallback(const nav_msgs::msg::Path::SharedPtr plan) {
    path_.poses.clear();
    clear_waypoints_ = true;
    path_.header = plan->header;
    path_.poses = plan->poses;
    createAndPublishMarkersFromPath(path_.poses);
    goal_pub_->publish(path_.poses.back());
}*/

void WaypointPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    // clear previous markers
    for (size_t i = 0; i < path.size(); i++)
    {
        marker.header = path[i].header;
        marker.ns = "global_planner";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = i;
        marker.pose.position = path[i].pose.position;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);
    }

    waypoint_marker_pub_->publish(markers);
}

} // namespace nav2_waypoint_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_waypoint_planner::WaypointPlanner, nav2_core::GlobalPlanner)