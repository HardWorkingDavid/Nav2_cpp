#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <fstream>
#include <iomanip>

class GlobalPlanLogger : public rclcpp::Node {
public:
    GlobalPlanLogger() : Node("global_plan_logger") {
        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&GlobalPlanLogger::planCallback, this, std::placeholders::_1));
        // Open CSV file for writing
        csv_file_.open("/home/bigdavid/Nav2/src/odom_plot/localization_data/global_plan.csv");
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open csv file.");
            //return;
        }
        // Write header to CSV file
        csv_file_ << "timestamp,x,y,z,qx,qy,qz,qw\n";

        RCLCPP_INFO(this->get_logger(), "GlobalPlanLogger node initialized.");
    }
private:
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        for (size_t i = 0; i < msg->poses.size(); i++) {
            // Extract timestamp
            auto timestamp = msg->header.stamp;
            // Extract position
            double x = msg->poses[i].pose.position.x;
            double y = msg->poses[i].pose.position.y;
            double z = msg->poses[i].pose.position.z;
            // Extract orientation
            double qx = msg->poses[i].pose.orientation.x;
            double qy = msg->poses[i].pose.orientation.y;
            double qz = msg->poses[i].pose.orientation.z;
            double qw = msg->poses[i].pose.orientation.w;
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
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    std::ofstream csv_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlanLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}