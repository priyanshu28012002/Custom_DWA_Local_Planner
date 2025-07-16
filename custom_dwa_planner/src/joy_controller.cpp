#include "custom_dwa_planner/joy_controller.hpp"

JoyServiceNode::JoyServiceNode() : Node("joy_controller_node"), start_(false)
{
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyServiceNode::joy_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    
    publish_cmd_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&JoyServiceNode::publish_cmd_vel, this));
}

void JoyServiceNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joystick_axes_ = *msg;
    start_ = true;
}

void JoyServiceNode::publish_cmd_vel()
{
    if(start_)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = joystick_axes_.axes[1] * 0.5;
        twist.angular.z = joystick_axes_.axes[2] * 0.5;
        cmd_vel_publisher_->publish(twist);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyServiceNode>());
    rclcpp::shutdown();
    return 0;
}