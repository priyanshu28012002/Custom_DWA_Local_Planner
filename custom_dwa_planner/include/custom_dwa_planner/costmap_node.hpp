
#ifndef GLOBAL_COSTMAP_HPP
#define GLOBAL_COSTMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <string>

class CostmapNode : public rclcpp::Node
{
public:
    CostmapNode();

private:
    // Callbacks
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Costmap functions
    void update_costmaps();
    void inflate_obstacle(int mx, int my, bool is_local);
    void publish_costmaps();

    // Parameters
    double map_width_;
    double map_height_;
    double resolution_;
    double global_map_width_;
    double global_map_height_;
    double local_map_height_;
    double global_width_px_;
    double global_height_px_;
    double local_map_width_;
    double inflation_radius_;
    std::string laser_frame_;
    
    // Global costmap parameters
    int width_px_;
    int height_px_;
    
    // Local costmap parameters (4x smaller area)
    int local_width_px_;
    int local_height_px_;
    double local_resolution_;
    
    // Costmap data
    std::vector<int8_t> global_costmap_;
    std::vector<int8_t> local_costmap_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    
    // ROS2 components
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr costmap_pub_timer_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif // GLOBAL_COSTMAP_HPP