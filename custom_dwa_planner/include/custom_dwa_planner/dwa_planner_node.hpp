#ifndef DWA_LOCAL_PLANNER_HPP
#define DWA_LOCAL_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <cmath>
#include <memory>
#include <utility>

class DWALocalPlanner : public rclcpp::Node
{
public:
    DWALocalPlanner();

private:
    void declare_parameters();
    void initialize_subscribers();
    void initialize_publishers();
    void movement_loop();
    void initialize_timer();
    std::pair<double, double> generate_sample();
    std::vector<std::pair<double, double>> predict_trajectory(double speed, double turn);
    double check_collision(const std::vector<std::pair<double, double>>& path);
    double cost_function(const std::vector<std::pair<double, double>>& path);
    double velocity_score(const std::vector<std::pair<double, double>>& path, double min_clearance) ;  
    double clearance_score(const std::vector<std::pair<double, double>>& path) ;
    double heading_to_goal_score(const std::pair<double, double>& path_end_point);

    std::pair<double, double> select_best_trajectory();
    void publish_candidate_trajectories(const std::vector<std::vector<std::pair<double, double>>>& candidates);
    void publish_selected_trajectory(const std::vector<std::pair<double, double>>& path);
    void publish_local_path(const std::vector<std::pair<double, double>>& path);
    double shortest_angular_distance(double from, double to);

    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr selected_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // Data storage
    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
    geometry_msgs::msg::PoseStamped::SharedPtr local_goal_;
    geometry_msgs::msg::PoseStamped::SharedPtr global_goal_;
    nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
    nav_msgs::msg::Path::SharedPtr global_plan_;
    
    bool goal_reached_;
    double goal_x_;
    double goal_y_;
    double goal_yaw_;  // Stored in radians
    geometry_msgs::msg::Quaternion goal_orientation_;
    bool use_local_goal_;

    // Parameters
    double max_speed_;
    double max_turn_;
    double step_time_;
    double goal_tolerance_;
    double safety_margin_;
    int sim_steps_;
    int num_samples_;
    double goal_weight_;
    double heading_weight_;
    double collision_penalty_;
    double smoothness_weight_;

    // Random number generation
    std::mt19937 random_generator_;
    std::uniform_real_distribution<double> speed_distribution_;
    std::uniform_real_distribution<double> turn_distribution_;

    // Callback methods
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void global_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg);

    // Timer callback method
    void control_loop_callback();

};

#endif