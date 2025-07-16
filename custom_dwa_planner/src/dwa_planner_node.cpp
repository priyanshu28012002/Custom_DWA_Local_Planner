#include "custom_dwa_planner/dwa_planner_node.hpp"

// todo
// joycontroller_setup setup joy to test the sensers odom and scan data || implenmetn kalanfilter
// costmap_setup Subscriber scan and publish global aswell as locl costmap || make map from global_costmap and implenmetn slam
// rrt global_planer_setup Subscriber goal , global_costmap and publish the global_plan || replan the path continious for better reconnection rrt*

// dwa_implementation Steps

// Get current state
// Generate velocity samples
// Gentnerate multipal trajectory 
// Pick trajectory with highest score of costfuncation base on 
// heading_to_goal_score toword the gole in correct orientation
// clearance_score obstrical in the costmap
// velocity_score 
// Select best velocity
// Publish the cmd_vel

DWALocalPlanner::DWALocalPlanner()
    : Node("dwa_local_planner"),
      random_generator_(std::random_device()()),
      goal_reached_(true),
      goal_x_(0.0),
      goal_y_(0.0)
      
{
    declare_parameters();
    initialize_subscribers();
    initialize_publishers();
    initialize_timer();

    speed_distribution_ = std::uniform_real_distribution<double>(0.0, max_speed_);
    turn_distribution_ = std::uniform_real_distribution<double>(-max_turn_, max_turn_);



    RCLCPP_INFO(get_logger(), "DWA Local Planner initialized");
}

void DWALocalPlanner::initialize_timer()
{
    control_loop_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(step_time_ * 1000)),
        std::bind(&DWALocalPlanner::control_loop_callback, this));
}

void DWALocalPlanner::control_loop_callback()
{
    if (goal_reached_ || !odom_data_ || !scan_data_) {
        return;
    }

    double x = odom_data_->pose.pose.position.x;
    double y = odom_data_->pose.pose.position.y;
    if (hypot(goal_x_ - x, goal_y_ - y) < goal_tolerance_) {
        goal_reached_ = true;
        RCLCPP_INFO(get_logger(), "Goal reached at (%.2f, %.2f)", goal_x_, goal_y_);
        return;
    }

    auto [v, w] = select_best_trajectory();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_vel_pub_->publish(cmd);
}

void DWALocalPlanner::declare_parameters()
{
    declare_parameter("max_speed", 0.15);
    declare_parameter("max_turn", 1.5);
    declare_parameter("step_time", 0.1);
    declare_parameter("goal_tolerance", 0.1);
    declare_parameter("safety_margin", 0.3);
    declare_parameter("sim_steps", 100);
    declare_parameter("num_samples", 100);
    declare_parameter("goal_weight", 5.0);
    declare_parameter("heading_weight", 2.0);
    declare_parameter("collision_penalty", 100000.0);
    declare_parameter("smoothness_weight", 0.1);
    max_speed_ = get_parameter("max_speed").as_double();
    max_turn_ = get_parameter("max_turn").as_double();
    step_time_ = get_parameter("step_time").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    sim_steps_ = get_parameter("sim_steps").as_int();
    num_samples_ = get_parameter("num_samples").as_int();
    goal_weight_ = get_parameter("goal_weight").as_double();
    heading_weight_ = get_parameter("heading_weight").as_double();
    collision_penalty_ = get_parameter("collision_penalty").as_double();
    smoothness_weight_ = get_parameter("smoothness_weight").as_double();
}

void DWALocalPlanner::initialize_subscribers()
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&DWALocalPlanner::odom_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DWALocalPlanner::scan_callback, this, std::placeholders::_1));

    global_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&DWALocalPlanner::global_goal_callback, this, std::placeholders::_1));

    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap", 10,
        std::bind(&DWALocalPlanner::local_costmap_callback, this, std::placeholders::_1));

    global_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&DWALocalPlanner::global_plan_callback, this, std::placeholders::_1));
}

void DWALocalPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_data_ = msg;
}

void DWALocalPlanner::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_data_ = msg;
}

void DWALocalPlanner::global_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    global_goal_ = msg;
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    
    // Store the orientation
    goal_orientation_ = msg->pose.orientation;
    
    // Convert quaternion to yaw
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goal_yaw_ = yaw;
    
    goal_reached_ = false;
    
    RCLCPP_INFO(get_logger(), "New goal received at (%.2f, %.2f) with yaw: %.2f radians", 
        goal_x_, goal_y_, goal_yaw_);
}

void DWALocalPlanner::local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    local_costmap_ = msg;
}

void DWALocalPlanner::global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_plan_ = msg;
}

void DWALocalPlanner::initialize_publishers()
{
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_trajectories", 10);
    selected_pub_ = create_publisher<visualization_msgs::msg::Marker>("/selected_trajectory", 10);
}

std::pair<double, double> DWALocalPlanner::generate_sample()
{
    return {speed_distribution_(random_generator_), turn_distribution_(random_generator_)};
}

std::vector<std::pair<double, double>> DWALocalPlanner::predict_trajectory(double speed, double turn)
{
    std::vector<std::pair<double, double>> path;
    if (!odom_data_)
        return path;

    double x = odom_data_->pose.pose.position.x;
    double y = odom_data_->pose.pose.position.y;
    tf2::Quaternion q(
        odom_data_->pose.pose.orientation.x,
        odom_data_->pose.pose.orientation.y,
        odom_data_->pose.pose.orientation.z,
        odom_data_->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    for (int i = 0; i < sim_steps_; ++i)
    {
        yaw += turn * step_time_;
        x += speed * cos(yaw) * step_time_;
        y += speed * sin(yaw) * step_time_;
        path.emplace_back(x, y);
    }
    return path;
}

double DWALocalPlanner::check_collision(const std::vector<std::pair<double, double>> &path)
{
    if (!scan_data_)
        return -INFINITY;

    for (const auto &[x, y] : path)
    {
        double dist = hypot(x, y);
        int idx = static_cast<int>((atan2(y, x) / (2 * M_PI)) * scan_data_->ranges.size());
        idx = std::max(0, std::min(static_cast<int>(scan_data_->ranges.size()) - 1, idx));
        if (dist < scan_data_->ranges[idx] - safety_margin_)
            return -collision_penalty_;
    }
    return 0;
}

double DWALocalPlanner::shortest_angular_distance(double from, double to)
{
    double diff = to - from;
    while (diff > M_PI)
        diff -= 2.0 * M_PI;
    while (diff < -M_PI)
        diff += 2.0 * M_PI;
    return diff;
}


double DWALocalPlanner::cost_function(const std::vector<std::pair<double, double>>& path) 
{
    if (path.empty() || !odom_data_ || !local_costmap_) {
        return -INFINITY;
    }

    // Goal distance score
    const auto& [x_end, y_end] = path.back();
    const double goal_dist = -std::hypot(goal_x_ - x_end, goal_y_ - y_end) * goal_weight_;
    
    // Calculate component scores
    const double heading_score = heading_to_goal_score(path.back());
    const double clearance = clearance_score(path);
    const double velocity = velocity_score(path, clearance);
    
    return goal_dist + heading_score + clearance + velocity;
}

double DWALocalPlanner::heading_to_goal_score(const std::pair<double, double>& path_point) 
{
    if (!odom_data_) return 0.0;
    
    const auto& [x_end, y_end] = path_point;
    const double goal_dx = goal_x_ - x_end;
    const double goal_dy = goal_y_ - y_end;
    
    const auto& orientation = odom_data_->pose.pose.orientation;
    const double yaw = 2.0 * atan2(orientation.z, orientation.w);
    const double target_heading = atan2(goal_dy, goal_dx);
    double heading_diff = shortest_angular_distance(yaw, target_heading);
    
    return -std::abs(heading_diff) * heading_weight_;
}


double DWALocalPlanner::clearance_score(const std::vector<std::pair<double, double>>& path) 
{
    if (!local_costmap_ || path.empty()) return 0.0;
    
    const auto& costmap_info = local_costmap_->info;
    const double resolution = costmap_info.resolution;
    const double origin_x = costmap_info.origin.position.x;
    const double origin_y = costmap_info.origin.position.y;
    
    double score = 0.0;
    double min_clearance = std::numeric_limits<double>::max();
    
    for (const auto& [x, y] : path) {
        const int mx = static_cast<int>((x - origin_x) / resolution);
        const int my = static_cast<int>((y - origin_y) / resolution);
        
        if (mx >= 0 && mx < static_cast<int>(costmap_info.width) &&
            my >= 0 && my < static_cast<int>(costmap_info.height)) {
            
            const int index = my * costmap_info.width + mx;
            const int8_t cost = local_costmap_->data[index];
            
            if (cost > 50) {
                score -= collision_penalty_;
            } else {
                const double clearance = (100 - cost) * resolution;
                min_clearance = std::min(min_clearance, clearance);
            }
        }
    }
    
    if (min_clearance < std::numeric_limits<double>::max()) {
        score += std::exp(min_clearance / safety_margin_) * 0.5;
    }
    
    return score;
}

double DWALocalPlanner::velocity_score(const std::vector<std::pair<double, double>>& path,double min_clearance) 
{
if (path.size() < 2) return 0.0;

const auto& [x1, y1] = path[0];
const auto& [x2, y2] = path[1];
const double velocity = std::hypot(x2 - x1, y2 - y1) / step_time_;

if (min_clearance > safety_margin_ * 0.5) {
return velocity * 0.1;
}
return 0.0;
}

std::pair<double, double> DWALocalPlanner::select_best_trajectory()
{
    double best_score = -INFINITY;
    std::pair<double, double> best_cmd{0.0, 0.0};
    std::vector<std::vector<std::pair<double, double>>> candidates;

    for (int i = 0; i < num_samples_; ++i)
    {
        auto [v, w] = generate_sample();
        auto traj = predict_trajectory(v, w);
        double score = cost_function(traj);
        if (score > best_score)
        {
            best_score = score;
            best_cmd = {v, w};
        }
        candidates.push_back(traj);
    }

    publish_candidate_trajectories(candidates);
    publish_selected_trajectory(predict_trajectory(best_cmd.first, best_cmd.second));
    publish_local_path(predict_trajectory(best_cmd.first, best_cmd.second));

    return best_cmd;
}

void DWALocalPlanner::publish_candidate_trajectories(const std::vector<std::vector<std::pair<double, double>>> &candidates)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &path : candidates)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = now();
        marker.ns = "candidates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;

        for (const auto &[x, y] : path)
        {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        marker_array.markers.push_back(marker);
    }
    candidates_pub_->publish(marker_array);
}

void DWALocalPlanner::publish_selected_trajectory(const std::vector<std::pair<double, double>> &path)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = now();
    marker.ns = "selected";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto &[x, y] : path)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        marker.points.push_back(p);
    }
    selected_pub_->publish(marker);
}

void DWALocalPlanner::publish_local_path(const std::vector<std::pair<double, double>> &path)
{
    nav_msgs::msg::Path local_path;
    local_path.header.frame_id = "odom";
    local_path.header.stamp = now();

    for (const auto &[x, y] : path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = local_path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        local_path.poses.push_back(pose);
    }
    local_path_pub_->publish(local_path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWALocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}