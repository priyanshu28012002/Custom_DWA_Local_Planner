#ifndef JOY_SERVICE_NODE_HPP
#define JOY_SERVICE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyServiceNode : public rclcpp::Node
{
public:
    JoyServiceNode();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish_cmd_vel();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr publish_cmd_vel_timer_;
    
    sensor_msgs::msg::Joy joystick_axes_;
    bool start_;
};

#endif