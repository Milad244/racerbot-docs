#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <algorithm>

class WallFollow : public rclcpp::Node {
public:
    WallFollow() : Node("wall_follow_node")
    {
        RCLCPP_INFO(this->get_logger(), "Wall follow node started");

        prev_time_ = this->now();

        /// TODO: Create ROS subscribers and publishers
    }

private:
    /// TODO: PID control parameters
    double kp_ = 0.0;
    double kd_ = 0.0;
    double ki_ = 0.0;
    double servo_offset_ = 0.0;
    rclcpp::Time prev_time_; // for dt
    double prev_error_ = 0.0; // for derivative
    double integral_ = 0.0; // for integral
    double desired_dist_ = 1.0;
    double look_ahead_ = 1.5;

    // Topics
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Returns the corresponding range measurement from a given angle
    double get_range(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        /// TODO: Implement, handling invalid ranges and indices
        return 0.0;
    }

    // Calculates the error to the wall, following the wall to the left (going counter clockwise in the Levine loop).
    double get_error(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /// TODO: Do the trigonometry for finding alpha
        // (you can pick any angle a as angle b + (0, 70] degrees) experiment!

        /// TODO: Use alpha to find and return the total error (error is with respect to desired_dist)
        return 0.0;
    }

    // Based on the given error, use PID to publish vehicle control
    void pid_control(double error)
    {
        /// TODO: Calculate steering angle using PID
        // (remember to clamp integral and steering angle)

        /// TODO: Calculate velocity based on steering angle

        /// TODO: Publish PID vehicle control
    }

    // Callback function for LaserScan messages. Calculates the error and calls PID control with it.
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: Implement
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
