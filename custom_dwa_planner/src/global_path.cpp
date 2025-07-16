#include "custom_dwa_planner/global_path.hpp"

//todo

// Start planning when both goal and odom are received
// Loop for max_iterations:
// Sample random point (bias goal every 5th iteration)
// Find nearest node
// Steer towards random point (new node)
// Check if the new path is collision-free
// Find nearby nodes (within search_radius)
// Choose best parent with least cost
// Rewire nearby nodes if cheaper path found
// If new node is within goal_tolerance, finish


RRTStarPlanner::RRTStarPlanner() : rclcpp::Node("global_rrt_star_planner")
{
    this->declare_parameter("max_iterations", 5000);
    this->declare_parameter("step_size", 0.5);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("search_radius", 1.0);
    this->declare_parameter("path_line_width", 0.1);

    max_iterations_ = this->get_parameter("max_iterations").as_int();
    step_size_ = this->get_parameter("step_size").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    search_radius_ = this->get_parameter("search_radius").as_double();
    path_line_width_ = this->get_parameter("path_line_width").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);
    path_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/global_plan_viz", 10);

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap", 10, std::bind(&RRTStarPlanner::costmap_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, std::bind(&RRTStarPlanner::goal_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&RRTStarPlanner::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RRT* Planner initialized");
}

void RRTStarPlanner::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    costmap_ = msg;
    resolution_ = costmap_->info.resolution;
    origin_x_ = costmap_->info.origin.position.x;
    origin_y_ = costmap_->info.origin.position.y;
    width_ = costmap_->info.width;
    height_ = costmap_->info.height;
}

void RRTStarPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_ = msg;
    if (costmap_ && odom_) {
        plan_path();
    }
}

void RRTStarPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = msg;
}

void RRTStarPlanner::plan_path()
{
    if (!costmap_ || !goal_ || !odom_) {
        RCLCPP_WARN(this->get_logger(), "Missing required data for planning");
        return;
    }

    try {
        auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

        std::vector<Node> tree;
        tree.push_back({
            transform.transform.translation.x,
            transform.transform.translation.y,
            0.0,
            -1
        });

        double goal_x = goal_->pose.position.x;
        double goal_y = goal_->pose.position.y;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> x_dist(
            costmap_->info.origin.position.x,
            costmap_->info.origin.position.x + width_ * resolution_);
        std::uniform_real_distribution<> y_dist(
            costmap_->info.origin.position.y,
            costmap_->info.origin.position.y + height_ * resolution_);

        bool path_found = false;

        for (int i = 0; i < max_iterations_; ++i) {
            double rand_x, rand_y;
            if (i % 5 == 0) {
                rand_x = goal_x;
                rand_y = goal_y;
            } else {
                rand_x = x_dist(gen);
                rand_y = y_dist(gen);
            }

            int nearest = find_nearest_node(tree, rand_x, rand_y);
            double theta = atan2(rand_y - tree[nearest].y, rand_x - tree[nearest].x);
            double new_x = tree[nearest].x + step_size_ * cos(theta);
            double new_y = tree[nearest].y + step_size_ * sin(theta);

            if (!is_collision_free(tree[nearest].x, tree[nearest].y, new_x, new_y)) {
                continue;
            }

            std::vector<int> nearby_nodes = find_nearby_nodes(tree, new_x, new_y, search_radius_);
            int best_parent = nearest;
            double best_cost = tree[nearest].cost + distance(tree[nearest].x, tree[nearest].y, new_x, new_y);

            for (int near : nearby_nodes) {
                double cost = tree[near].cost + distance(tree[near].x, tree[near].y, new_x, new_y);
                if (cost < best_cost && is_collision_free(tree[near].x, tree[near].y, new_x, new_y)) {
                    best_parent = near;
                    best_cost = cost;
                }
            }

            Node new_node = {new_x, new_y, best_cost, best_parent};
            tree.push_back(new_node);

            for (int near : nearby_nodes) {
                double cost = new_node.cost + distance(new_node.x, new_node.y, tree[near].x, tree[near].y);
                if (cost < tree[near].cost && is_collision_free(new_node.x, new_node.y, tree[near].x, tree[near].y)) {
                    tree[near].parent = tree.size() - 1;
                    tree[near].cost = cost;
                }
            }

            if (distance(new_x, new_y, goal_x, goal_y) < goal_tolerance_) {
                path_found = true;
                break;
            }
        }

        if (path_found) {
            nav_msgs::msg::Path path;
            path.header.stamp = this->now();
            path.header.frame_id = "odom";

            int current_idx = tree.size() - 1;
            while (current_idx >= 0) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = tree[current_idx].x;
                pose.pose.position.y = tree[current_idx].y;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
                current_idx = tree[current_idx].parent;
            }

            std::reverse(path.poses.begin(), path.poses.end());
            path_pub_->publish(path);
            publish_path_visualization(path);
            RCLCPP_INFO(this->get_logger(), "Path found and published");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to find path within iterations");
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
    }
}

void RRTStarPlanner::publish_path_visualization(const nav_msgs::msg::Path& path)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = path_line_width_;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    for (const auto& pose : path.poses) {
        geometry_msgs::msg::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.01;
        marker.points.push_back(p);
    }

    path_viz_pub_->publish(marker);
}

int RRTStarPlanner::find_nearest_node(const std::vector<Node>& tree, double x, double y)
{
    int nearest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tree.size(); ++i) {
        double dist = distance(tree[i].x, tree[i].y, x, y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = i;
        }
    }

    return nearest;
}

std::vector<int> RRTStarPlanner::find_nearby_nodes(const std::vector<Node>& tree, double x, double y, double radius)
{
    std::vector<int> nearby;

    for (size_t i = 0; i < tree.size(); ++i) {
        if (distance(tree[i].x, tree[i].y, x, y) < radius) {
            nearby.push_back(i);
        }
    }

    return nearby;
}

bool RRTStarPlanner::is_collision_free(double x1, double y1, double x2, double y2)
{
    if (!costmap_) return false;

    int mx1 = static_cast<int>((x1 - origin_x_) / resolution_);
    int my1 = static_cast<int>((y1 - origin_y_) / resolution_);
    int mx2 = static_cast<int>((x2 - origin_x_) / resolution_);
    int my2 = static_cast<int>((y2 - origin_y_) / resolution_);

    int dx = abs(mx2 - mx1);
    int dy = -abs(my2 - my1);
    int sx = mx1 < mx2 ? 1 : -1;
    int sy = my1 < my2 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (mx1 < 0 || mx1 >= width_ || my1 < 0 || my1 >= height_) {
            return false;
        }

        int index = my1 * width_ + mx1;
        if (costmap_->data[index] > 50) {
            return false;
        }

        if (mx1 == mx2 && my1 == my2) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            mx1 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            my1 += sy;
        }
    }

    return true;
}

double RRTStarPlanner::distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}