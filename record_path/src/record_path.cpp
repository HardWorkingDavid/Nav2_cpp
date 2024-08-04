#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iomanip>

const double RECORD_DISTANCE = 0.04;
double last_x_ = 0.0;
double last_y_ = 0.0;

class RecordPath : public rclcpp::Node {
public:
    RecordPath() : Node("record_path") {
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RecordPath::odomCallback, this, std::placeholders::_1));
        // Open CSV file for writing
        csv_file_.open("/home/bigdavid/Nav2/src/record_path/path_data/path_data.csv");
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open csv file.");
            return;
        }

        // Write header to CSV file
        csv_file_ << "x,y,z,qx,qy,qz,qw\n";
        RCLCPP_INFO(this->get_logger(), "record_path node initialized.");
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        double distance = std::hypot(current_x - last_x_, current_y - last_y_);

        if (distance >= RECORD_DISTANCE) {
            last_x_ = current_x;
            last_y_ = current_y;

            double z = 0.0;
            // Extract orientation
            double qx = 0.0;
            double qy = 0.0;
            double qz = 0.0;
            double qw = 1.0;

            // Write data to CSV file
            csv_file_ << std::fixed << std::setprecision(6)
                      << current_x << " "
                      << current_y << " "
                      << z << " "
                      << qx << " "
                      << qy << " "
                      << qz << " "
                      << qw << "\n";
            csv_file_.flush(); // 立即刷新缓冲区，确保数据实时写入文件
        }
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    std::ofstream csv_file_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordPath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}