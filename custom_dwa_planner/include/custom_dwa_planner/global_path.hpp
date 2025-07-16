#ifndef RRT_STAR_PLANNER_HPP
#define RRT_STAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

class RRTStarPlanner : public rclcpp::Node
{
public:
    RRTStarPlanner();

private:
    struct Node {
        double x;
        double y;
        double cost;
        int parent;
    };

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void plan_path();
    void publish_path_visualization(const nav_msgs::msg::Path& path);
    int find_nearest_node(const std::vector<Node>& tree, double x, double y);
    std::vector<int> find_nearby_nodes(const std::vector<Node>& tree, double x, double y, double radius);
    bool is_collision_free(double x1, double y1, double x2, double y2);
    double distance(double x1, double y1, double x2, double y2);

    int max_iterations_;
    double step_size_;
    double goal_tolerance_;
    double search_radius_;
    double path_line_width_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_viz_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif