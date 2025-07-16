
#include "custom_dwa_planner/costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    this->declare_parameter("global_map_width", 10.0);  
    this->declare_parameter("global_map_height", 10.0); 
    this->declare_parameter("local_map_width", 2.5);    
    this->declare_parameter("local_map_height", 2.5);   
    this->declare_parameter("resolution", 0.05);        
    this->declare_parameter("inflation_radius", 0.3);   
    this->declare_parameter("laser_frame", "base_scan");
    
    global_map_width_ = this->get_parameter("global_map_width").as_double();
    global_map_height_ = this->get_parameter("global_map_height").as_double();
    local_map_width_ = this->get_parameter("local_map_width").as_double();
    local_map_height_ = this->get_parameter("local_map_height").as_double();
    resolution_ = this->get_parameter("resolution").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    
    global_width_px_ = static_cast<int>(global_map_width_ / resolution_);
    global_height_px_ = static_cast<int>(global_map_height_ / resolution_);
    local_width_px_ = static_cast<int>(local_map_width_ / resolution_);
    local_height_px_ = static_cast<int>(local_map_height_ / resolution_);
    
    global_costmap_.resize(global_width_px_ * global_height_px_, 0);
    local_costmap_.resize(local_width_px_ * local_height_px_, 0);
    
    global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap", 10);
    local_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_costmap", 10);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&CostmapNode::scan_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&CostmapNode::odom_callback, this, std::placeholders::_1));
        
    costmap_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),  
        std::bind(&CostmapNode::publish_costmaps, this));
        
    RCLCPP_INFO(this->get_logger(), "Costmap node initialized");
    RCLCPP_INFO(this->get_logger(), "Global costmap size: %dx%d px", width_px_, height_px_);
    RCLCPP_INFO(this->get_logger(), "Local costmap size: %dx%d px", local_width_px_, local_height_px_);
}

void CostmapNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    if (!msg->header.frame_id.empty()) {
        laser_frame_ = msg->header.frame_id;
    }
    update_costmaps();
}

void CostmapNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = msg;
    update_costmaps();
}

void CostmapNode::inflate_obstacle(int mx, int my, bool is_local)
{
    int inflation_cells = static_cast<int>(inflation_radius_ / 
        (is_local ? local_resolution_ : resolution_));
    
    auto& costmap = is_local ? local_costmap_ : global_costmap_;
    int width = is_local ? local_width_px_ : width_px_;
    int height = is_local ? local_height_px_ : height_px_;

    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            int nx = mx + dx;
            int ny = my + dy;

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }

            float distance = sqrt(dx*dx + dy*dy) * (is_local ? local_resolution_ : resolution_);
            if (distance > inflation_radius_) {
                continue;
            }

            int index = ny * width + nx;
            int cost = static_cast<int>(80 * (1.0 - distance/inflation_radius_));

            if (cost > costmap[index] && costmap[index] < 100) {
                costmap[index] = cost;
            }
        }
    }
}


void CostmapNode::update_costmaps()
{
    if (!latest_scan_) {
        return;
    }
    
    std::fill(global_costmap_.begin(), global_costmap_.end(), 0);
    std::fill(local_costmap_.begin(), local_costmap_.end(), 0);
    
    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        float range = latest_scan_->ranges[i];
        
        if (range < latest_scan_->range_min || range > latest_scan_->range_max) {
            continue;
        }
        
        float angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
        float x_laser = range * cos(angle);
        float y_laser = range * sin(angle);
        
        int local_mx = static_cast<int>((x_laser + local_map_width_/2) / resolution_);
        int local_my = static_cast<int>((y_laser + local_map_height_/2) / resolution_);
        
        if (local_mx >= 0 && local_mx < local_width_px_ && 
            local_my >= 0 && local_my < local_height_px_) {
            int local_index = local_my * local_width_px_ + local_mx;
            local_costmap_[local_index] = 100;
            inflate_obstacle(local_mx, local_my, true);
        }
    }
    
    if (latest_odom_) {
        try {
            if (!tf_buffer_.canTransform("odom", laser_frame_, tf2::TimePointZero)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for transform from %s to odom",
                    laser_frame_.c_str());
                return;
            }
            
            auto transform = tf_buffer_.lookupTransform("odom", laser_frame_, tf2::TimePointZero);
            
            for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
                float range = latest_scan_->ranges[i];
                
                if (range < latest_scan_->range_min || range > latest_scan_->range_max) {
                    continue;
                }
                
                float angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
                float x_laser = range * cos(angle);
                float y_laser = range * sin(angle);
                
                tf2::Vector3 point_laser(x_laser, y_laser, 0);
                tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                tf2::Transform tf_odom_laser(q, tf2::Vector3(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z));

                tf2::Vector3 point_odom = tf_odom_laser * point_laser;
                
                int global_mx = static_cast<int>((point_odom.x() + global_map_width_/2) / resolution_);
                int global_my = static_cast<int>((point_odom.y() + global_map_height_/2) / resolution_);
                
                if (global_mx >= 0 && global_mx < global_width_px_ && 
                    global_my >= 0 && global_my < global_height_px_) {
                    int global_index = global_my * global_width_px_ + global_mx;
                    global_costmap_[global_index] = 100;
                    inflate_obstacle(global_mx, global_my, false);
                }
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
        }
    }
}

void CostmapNode::publish_costmaps()
{
    auto global_msg = nav_msgs::msg::OccupancyGrid();
    global_msg.header.stamp = this->now();
    global_msg.header.frame_id = "odom";
    global_msg.info.resolution = resolution_;
    global_msg.info.width = global_width_px_;
    global_msg.info.height = global_height_px_;
    global_msg.info.origin.position.x = -global_map_width_/2;
    global_msg.info.origin.position.y = -global_map_height_/2;
    global_msg.info.origin.position.z = 0.0;
    global_msg.info.origin.orientation.w = 1.0;
    global_msg.data = global_costmap_;
    global_costmap_pub_->publish(global_msg);
    
    auto local_msg = nav_msgs::msg::OccupancyGrid();
    local_msg.header.stamp = this->now();
    local_msg.header.frame_id = laser_frame_;
    local_msg.info.resolution = resolution_;
    local_msg.info.width = local_width_px_;
    local_msg.info.height = local_height_px_;
    local_msg.info.origin.position.x = -local_map_width_/2;  // Centered on laser
    local_msg.info.origin.position.y = -local_map_height_/2;
    local_msg.info.origin.position.z = 0.0;
    local_msg.info.origin.orientation.w = 1.0;
    local_msg.data = local_costmap_;
    local_costmap_pub_->publish(local_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}