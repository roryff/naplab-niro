#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <vector>
#include <mutex>

/**
 * @brief Simple path structure for recording and following
 */
struct PathPoint {
    double x;        // Local ENU x (east)
    double y;        // Local ENU y (north)
    double heading;  // Heading in radians
    double lat;      // Original latitude
    double lon;      // Original longitude
};

/**
 * @brief ROS2 Path Follower Node for autonomous vehicle control
 * 
 * This node records and follows paths using Stanley controller.
 * Works with gnss_node for position and comma_node for vehicle control.
 * 
 * Subscriptions:
 *   - gnss/odometry (nav_msgs/Odometry): Current vehicle pose and velocity
 *   - gnss/fix (sensor_msgs/NavSatFix): GPS coordinates for path recording
 *   - vehicle/steering_angle (std_msgs/Float64): Current steering wheel angle [deg]
 *   - vehicle/speed (std_msgs/Float64): Current vehicle speed [km/h]
 *   - path/enable_recording (std_msgs/Bool): Start/stop path recording
 *   - path/enable_following (std_msgs/Bool): Start/stop path following
 * 
 * Publications:
 *   - control/steering_angle (std_msgs/Float64): Desired steering angle [deg]
 *   - control/speed (std_msgs/Float64): Desired speed [km/h]
 *   - path/following_active (std_msgs/Bool): Path following status
 *   - path/visualization (nav_msgs/Path): Path for visualization
 *   - path/lateral_error (std_msgs/Float64): Cross-track error [m]
 *   - path/heading_error (std_msgs/Float64): Heading error [rad]
 */
class PathFollowerNode : public rclcpp::Node
{
public:
    PathFollowerNode() : Node("path_follower_node"), 
                         path_(),
                         closest_point_idx_(0),
                         recording_(false),
                         following_(false)
    {
        // Declare parameters
        this->declare_parameter("wheelbase", 2.7);  // meters
        this->declare_parameter("max_steering_angle_deg", 30.0);
        this->declare_parameter("max_steering_wheel_angle_deg", 460.0);
        this->declare_parameter("lookahead_distance", 5.0);  // meters
        this->declare_parameter("stanley_k_e", 0.5);  // Cross-track error gain
        this->declare_parameter("stanley_k_v", 1.0);  // Softening term
        this->declare_parameter("max_speed_kmh", 30.0);
        this->declare_parameter("min_recording_distance", 2.0);  // Min distance between recorded points
        
        // Get parameters
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle_deg").as_double() * M_PI / 180.0;
        max_steering_wheel_angle_ = this->get_parameter("max_steering_wheel_angle_deg").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        k_e_ = this->get_parameter("stanley_k_e").as_double();
        k_v_ = this->get_parameter("stanley_k_v").as_double();
        max_speed_ = this->get_parameter("max_speed_kmh").as_double();
        min_recording_distance_ = this->get_parameter("min_recording_distance").as_double();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gnss/odometry", 10,
            std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        
        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gnss/fix", 10,
            std::bind(&PathFollowerNode::fixCallback, this, std::placeholders::_1));
        
        steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle/steering_angle", 10,
            std::bind(&PathFollowerNode::steeringCallback, this, std::placeholders::_1));
        
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle/speed", 10,
            std::bind(&PathFollowerNode::speedCallback, this, std::placeholders::_1));
        
        record_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "path/enable_recording", 10,
            std::bind(&PathFollowerNode::recordCallback, this, std::placeholders::_1));
        
        follow_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "path/enable_following", 10,
            std::bind(&PathFollowerNode::followCallback, this, std::placeholders::_1));
        
        // Publishers
        steering_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "control/steering_angle", 10);
        speed_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "control/speed", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "path/following_active", 10);
        path_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "path/visualization", 10);
        lateral_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "path/lateral_error", 10);
        heading_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "path/heading_error", 10);
        
        // Control timer (20 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PathFollowerNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Follower Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Wheelbase: %.2f m", wheelbase_);
        RCLCPP_INFO(this->get_logger(), "  Max steering: %.1f deg", max_steering_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  Lookahead: %.2f m", lookahead_distance_);
    }

private:
    /**
     * @brief Main control loop - runs at 20 Hz
     */
    void controlLoop()
    {
        if (!following_ || path_.empty()) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Find closest point on path
        findClosestPoint();
        
        // Calculate errors
        double cross_track_error = calculateCrossTrackError();
        double heading_error = calculateHeadingError();
        
        // Stanley controller
        double steering_angle = stanleyControl(cross_track_error, heading_error);
        
        // Publish commands
        auto steering_msg = std_msgs::msg::Float64();
        auto speed_msg = std_msgs::msg::Float64();
        
        // Convert steering angle to steering wheel angle
        double steering_wheel_angle = (steering_angle * 180.0 / M_PI) * 
                                       (max_steering_wheel_angle_ / (max_steering_angle_ * 180.0 / M_PI));
        steering_msg.data = steering_wheel_angle;  // degrees
        speed_msg.data = max_speed_;  // km/h
        
        steering_cmd_pub_->publish(steering_msg);
        speed_cmd_pub_->publish(speed_msg);
        
        // Publish errors for monitoring
        auto lat_err_msg = std_msgs::msg::Float64();
        auto head_err_msg = std_msgs::msg::Float64();
        lat_err_msg.data = cross_track_error;
        head_err_msg.data = heading_error;
        lateral_error_pub_->publish(lat_err_msg);
        heading_error_pub_->publish(head_err_msg);
        
        // Check if we've reached the end
        if (closest_point_idx_ >= path_.size() - 5) {
            RCLCPP_INFO(this->get_logger(), "Path end reached, stopping");
            stopFollowing();
        }
    }
    
    /**
     * @brief Stanley lateral controller
     */
    double stanleyControl(double cross_track_error, double heading_error)
    {
        // Stanley control law: δ = ψ + arctan(k_e * e / (k_v + v))
        double v = current_speed_ / 3.6;  // Convert km/h to m/s
        
        double steering_angle = heading_error + 
                                std::atan2(k_e_ * cross_track_error, k_v_ + v);
        
        // Clamp to limits
        steering_angle = std::max(-max_steering_angle_, 
                                  std::min(max_steering_angle_, steering_angle));
        
        return steering_angle;
    }
    
    /**
     * @brief Find closest point on path to current position
     */
    void findClosestPoint()
    {
        double min_dist = std::numeric_limits<double>::max();
        size_t search_start = (closest_point_idx_ > 10) ? closest_point_idx_ - 10 : 0;
        size_t search_end = std::min(closest_point_idx_ + 50, path_.size());
        
        for (size_t i = search_start; i < search_end; ++i) {
            double dx = path_[i].x - current_x_;
            double dy = path_[i].y - current_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_point_idx_ = i;
            }
        }
    }
    
    /**
     * @brief Calculate cross-track error (lateral distance from path)
     */
    double calculateCrossTrackError()
    {
        if (closest_point_idx_ >= path_.size()) return 0.0;
        
        const auto& closest = path_[closest_point_idx_];
        
        // Vector from closest point to vehicle
        double dx = current_x_ - closest.x;
        double dy = current_y_ - closest.y;
        
        // Path direction at closest point
        double path_heading = closest.heading;
        
        // Cross-track error is perpendicular distance
        // Positive error means vehicle is to the right of the path
        double error = -dx * std::sin(path_heading) + dy * std::cos(path_heading);
        
        return error;
    }
    
    /**
     * @brief Calculate heading error relative to path
     */
    double calculateHeadingError()
    {
        if (closest_point_idx_ >= path_.size()) return 0.0;
        
        double path_heading = path_[closest_point_idx_].heading;
        double error = path_heading - current_heading_;
        
        // Normalize to [-pi, pi]
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;
        
        return error;
    }
    
    /**
     * @brief Odometry callback - updates current vehicle state
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_heading_ = std::atan2(2.0 * (qw*qz + qx*qy), 
                                      1.0 - 2.0 * (qy*qy + qz*qz));
        
        // Record path if enabled
        if (recording_) {
            recordPoint();
        }
        
        // Publish path visualization periodically
        static int viz_counter = 0;
        if (++viz_counter >= 10) {
            publishPathVisualization();
            viz_counter = 0;
        }
    }
    
    /**
     * @brief GPS fix callback - saves lat/lon with path points
     */
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
    }
    
    /**
     * @brief Steering angle callback
     */
    void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_steering_angle_ = msg->data * M_PI / 180.0;  // Convert to radians
    }
    
    /**
     * @brief Speed callback
     */
    void speedCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_speed_ = msg->data;  // km/h
    }
    
    /**
     * @brief Record path callback
     */
    void recordCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !recording_ && !following_) {
            // Start recording
            path_.clear();
            recording_ = true;
            RCLCPP_INFO(this->get_logger(), "Started path recording");
        } else if (!msg->data && recording_) {
            // Stop recording
            recording_ = false;
            RCLCPP_INFO(this->get_logger(), "Stopped path recording. Recorded %zu points", path_.size());
        }
    }
    
    /**
     * @brief Follow path callback
     */
    void followCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !following_ && !recording_) {
            // Start following
            if (path_.empty()) {
                RCLCPP_WARN(this->get_logger(), "Cannot start following: path is empty");
                return;
            }
            
            following_ = true;
            closest_point_idx_ = 0;
            
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = true;
            status_pub_->publish(status_msg);
            
            RCLCPP_INFO(this->get_logger(), "Started path following with %zu points", path_.size());
        } else if (!msg->data && following_) {
            stopFollowing();
        }
    }
    
    /**
     * @brief Stop path following
     */
    void stopFollowing()
    {
        following_ = false;
        
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = false;
        status_pub_->publish(status_msg);
        
        // Send zero commands
        auto steering_msg = std_msgs::msg::Float64();
        auto speed_msg = std_msgs::msg::Float64();
        steering_msg.data = 0.0;
        speed_msg.data = 0.0;
        steering_cmd_pub_->publish(steering_msg);
        speed_cmd_pub_->publish(speed_msg);
        
        RCLCPP_INFO(this->get_logger(), "Stopped path following");
    }
    
    /**
     * @brief Record a point on the path
     */
    void recordPoint()
    {
        // Only record if we've moved enough distance from last point
        if (!path_.empty()) {
            const auto& last = path_.back();
            double dx = current_x_ - last.x;
            double dy = current_y_ - last.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_recording_distance_) {
                return;  // Too close to last point
            }
        }
        
        PathPoint point;
        point.x = current_x_;
        point.y = current_y_;
        point.heading = current_heading_;
        point.lat = current_lat_;
        point.lon = current_lon_;
        
        path_.push_back(point);
        
        if (path_.size() % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Recorded %zu points", path_.size());
        }
    }
    
    /**
     * @brief Publish path visualization
     */
    void publishPathVisualization()
    {
        if (path_.empty()) return;
        
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& point : path_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            
            // Convert heading to quaternion
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(point.heading / 2.0);
            pose.pose.orientation.w = std::cos(point.heading / 2.0);
            
            path_msg.poses.push_back(pose);
        }
        
        path_viz_pub_->publish(path_msg);
    }
    
    // ROS2 communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Path data
    std::vector<PathPoint> path_;
    size_t closest_point_idx_;
    
    // State
    std::mutex state_mutex_;
    bool recording_;
    bool following_;
    
    // Current vehicle state
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_heading_ = 0.0;
    double current_lat_ = 0.0;
    double current_lon_ = 0.0;
    double current_speed_ = 0.0;  // km/h
    double current_steering_angle_ = 0.0;  // radians
    
    // Parameters
    double wheelbase_;
    double max_steering_angle_;
    double max_steering_wheel_angle_;
    double lookahead_distance_;
    double k_e_;  // Stanley cross-track gain
    double k_v_;  // Stanley softening term
    double max_speed_;
    double min_recording_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
