#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Pose {
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qw;
};

std::vector<Pose> read_csv_file(const std::string& filename) {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        return poses;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == 'timestamp,x,y,z,qx,qy,qz,qw') continue;
        std::istringstream ss(line);
        Pose pose;
        ss >> pose.timestamp >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >> pose.qz >> pose.qw;
        poses.push_back(pose);
    }
    file.close();
    return poses;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("tum_trajectory_visualizer");
    if (argc != 2)
    {
        RCLCPP_ERROR(nh->get_logger(), "Usage: ros2 run tum_trajectory_visualizer visualize_trajectory <path_to_tum_file>");
        return -1;
    }
    std::string csv_file = argv[1]; //"/home/bigdavid/Nav2/src/odom_plot/localization_data/odometry_data.csv";
    std::vector<Pose> poses = read_csv_file(csv_file);
    auto marker_pub = nh->create_publisher<visualization_msgs::msg::Marker>("/visual_odom", 10);

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = nh->now();
    line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = 0.05;
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    size_t current_index = 0;
    rclcpp::Rate rate(50); 
    while (rclcpp::ok() && current_index < poses.size())
    {
        geometry_msgs::msg::Point p;
        p.x = poses[current_index].x;
        p.y = poses[current_index].y;
        p.z = poses[current_index].z;
        line_strip.points.push_back(p);

        line_strip.header.stamp = nh->now(); // Use current time, or use poses[current_index].timestamp if available
        marker_pub->publish(line_strip);

        current_index++;
        rclcpp::spin_some(nh);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}