#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iomanip>

class OdometryLogger : public rclcpp::Node {
public:
    OdometryLogger() : Node("odometry_logger") {
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdometryLogger::odomCallback, this, std::placeholders::_1));
        // Open CSV file for writing
        csv_file_.open("/home/bigdavid/Nav2/src/odom_plot/localization_data/odometry_data.csv");
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open csv file.");
            //return;
        }
        // Write header to CSV file
        csv_file_ << "timestamp,x,y,z,qx,qy,qz,qw\n";

        RCLCPP_INFO(this->get_logger(), "OdometryLogger node initialized.");
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract timestamp
        auto timestamp = msg->header.stamp;
        // Extract position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        // Extract orientation
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        // Write data to CSV file
        csv_file_ << std::fixed << std::setprecision(6)
                  << timestamp.sec << "." << timestamp.nanosec << " "
                  << x << " "
                  << y << " "
                  << z << " "
                  << qx << " "
                  << qy << " "
                  << qz << " "
                  << qw << "\n";
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    std::ofstream csv_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}