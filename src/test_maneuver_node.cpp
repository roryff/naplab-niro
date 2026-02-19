#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>

/**
 * @brief Test Maneuver Node for evaluating vehicle control actuators
 * 
 * Generates predefined test trajectories (S-curves, circles, etc.) to test
 * the steering and speed control systems in a parking lot or controlled area.
 * 
 * Subscriptions:
 *   - gnss/odometry (nav_msgs/Odometry): Current vehicle state
 *   - test/start_maneuver (std_msgs/String): Maneuver type to execute
 * 
 * Publications:
 *   - control/steering_angle (std_msgs/Float64): Desired steering [deg]
 *   - control/speed (std_msgs/Float64): Desired speed [km/h]
 *   - test/trajectory (nav_msgs/Path): Test trajectory visualization
 *   - test/status (std_msgs/String): Current test status
 *   - test/active (std_msgs/Bool): Test maneuver active
 */
class TestManeuverNode : public rclcpp::Node
{
public:
    TestManeuverNode() : Node("test_maneuver_node"),
                         maneuver_active_(false),
                         path_index_(0)
    {
        // Parameters
        this->declare_parameter("wheelbase", 2.7);
        this->declare_parameter("max_steering_angle_deg", 30.0);
        this->declare_parameter("max_steering_wheel_angle_deg", 460.0);
        
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle_deg").as_double() * M_PI / 180.0;
        max_steering_wheel_angle_ = this->get_parameter("max_steering_wheel_angle_deg").as_double();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gnss/odometry", 10,
            std::bind(&TestManeuverNode::odomCallback, this, std::placeholders::_1));
        
        maneuver_sub_ = this->create_subscription<std_msgs::msg::String>(
            "test/start_maneuver", 10,
            std::bind(&TestManeuverNode::maneuverCallback, this, std::placeholders::_1));
        
        // Publishers
        steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("control/steering_angle", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("control/speed", 10);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("test/trajectory", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("test/status", 10);
        active_pub_ = this->create_publisher<std_msgs::msg::Bool>("test/active", 10);
        
        // Control timer (20 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TestManeuverNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Test Maneuver Node initialized");
        RCLCPP_INFO(this->get_logger(), "Available maneuvers:");
        RCLCPP_INFO(this->get_logger(), "  - 's_curve_5m' : Small S-curve (5m amplitude)");
        RCLCPP_INFO(this->get_logger(), "  - 's_curve_10m': Medium S-curve (10m amplitude)");
        RCLCPP_INFO(this->get_logger(), "  - 'circle_left': Circle left (10m radius)");
        RCLCPP_INFO(this->get_logger(), "  - 'circle_right': Circle right (10m radius)");
        RCLCPP_INFO(this->get_logger(), "  - 'straight': Straight line (50m)");
        RCLCPP_INFO(this->get_logger(), "  - 'stop': Stop current maneuver");
        
        publishStatus("Waiting for maneuver command");
    }

private:
    void controlLoop()
    {
        if (!maneuver_active_ || test_path_.empty()) {
            return;
        }
        
        // Find closest point on test path
        findClosestPoint();
        
        // Calculate control based on test trajectory
        double cross_track_error = calculateCrossTrackError();
        double heading_error = calculateHeadingError();
        
        // Simple controller
        double steering_angle = heading_error + std::atan2(0.5 * cross_track_error, 1.0 + current_speed_/3.6);
        steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));
        
        // Get desired speed from test path
        double desired_speed = test_path_[path_index_].speed;
        
        // Publish commands
        auto steering_msg = std_msgs::msg::Float64();
        auto speed_msg = std_msgs::msg::Float64();
        
        double steering_wheel_angle = (steering_angle * 180.0 / M_PI) * 
                                       (max_steering_wheel_angle_ / (max_steering_angle_ * 180.0 / M_PI));
        steering_msg.data = steering_wheel_angle;
        speed_msg.data = desired_speed;
        
        steering_pub_->publish(steering_msg);
        speed_pub_->publish(speed_msg);
        
        // Check if maneuver complete
        if (path_index_ >= test_path_.size() - 3) {
            RCLCPP_INFO(this->get_logger(), "Test maneuver complete!");
            stopManeuver();
        }
    }
    
    void maneuverCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string maneuver = msg->data;
        
        if (maneuver == "stop") {
            stopManeuver();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting maneuver: %s", maneuver.c_str());
        
        // Generate test trajectory based on requested maneuver
        if (maneuver == "s_curve_5m") {
            generateSCurve(50.0, 5.0, 15.0);
        } else if (maneuver == "s_curve_10m") {
            generateSCurve(80.0, 10.0, 15.0);
        } else if (maneuver == "circle_left") {
            generateCircle(10.0, true, 15.0);
        } else if (maneuver == "circle_right") {
            generateCircle(10.0, false, 15.0);
        } else if (maneuver == "straight") {
            generateStraight(50.0, 20.0);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown maneuver: %s", maneuver.c_str());
            publishStatus("Unknown maneuver");
            return;
        }
        
        maneuver_active_ = true;
        path_index_ = 0;
        
        auto active_msg = std_msgs::msg::Bool();
        active_msg.data = true;
        active_pub_->publish(active_msg);
        
        publishTrajectory();
        publishStatus("Executing: " + maneuver);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_speed_ = std::sqrt(
            msg->twist.twist.linear.x * msg->twist.twist.linear.x +
            msg->twist.twist.linear.y * msg->twist.twist.linear.y
        ) * 3.6;  // Convert to km/h
        
        // Extract yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_heading_ = std::atan2(2.0 * (qw*qz + qx*qy), 
                                      1.0 - 2.0 * (qy*qy + qz*qz));
    }
    
    void generateSCurve(double length, double amplitude, double speed_kmh)
    {
        test_path_.clear();
        
        double wavelength = length / 2.0;  // Two half-cycles for S-curve
        int num_points = static_cast<int>(length / 1.0);  // 1m spacing
        
        for (int i = 0; i < num_points; i++) {
            double s = (i * length) / (num_points - 1);
            
            TestPoint point;
            point.x = current_x_ + s * std::cos(current_heading_);
            point.y = current_y_ + s * std::sin(current_heading_) + 
                     amplitude * std::sin(2.0 * M_PI * s / wavelength);
            point.heading = current_heading_ + 
                           (2.0 * M_PI * amplitude / wavelength) * 
                           std::cos(2.0 * M_PI * s / wavelength);
            point.speed = speed_kmh;
            
            test_path_.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated S-curve: length=%.1fm, amplitude=%.1fm, %zu points",
                    length, amplitude, test_path_.size());
    }
    
    void generateCircle(double radius, bool left, double speed_kmh)
    {
        test_path_.clear();
        
        double circumference = 2.0 * M_PI * radius;
        int num_points = static_cast<int>(circumference / 1.0);  // 1m spacing
        
        double center_x = current_x_ + radius * std::cos(current_heading_ + (left ? M_PI/2 : -M_PI/2));
        double center_y = current_y_ + radius * std::sin(current_heading_ + (left ? M_PI/2 : -M_PI/2));
        
        double start_angle = std::atan2(current_y_ - center_y, current_x_ - center_x);
        
        for (int i = 0; i < num_points; i++) {
            double angle = start_angle + (left ? 1.0 : -1.0) * (2.0 * M_PI * i / num_points);
            
            TestPoint point;
            point.x = center_x + radius * std::cos(angle);
            point.y = center_y + radius * std::sin(angle);
            point.heading = angle + (left ? M_PI/2 : -M_PI/2);
            point.speed = speed_kmh;
            
            test_path_.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated circle: radius=%.1fm, direction=%s, %zu points",
                    radius, left ? "left" : "right", test_path_.size());
    }
    
    void generateStraight(double length, double speed_kmh)
    {
        test_path_.clear();
        
        int num_points = static_cast<int>(length / 1.0);  // 1m spacing
        
        for (int i = 0; i < num_points; i++) {
            double s = (i * length) / (num_points - 1);
            
            TestPoint point;
            point.x = current_x_ + s * std::cos(current_heading_);
            point.y = current_y_ + s * std::sin(current_heading_);
            point.heading = current_heading_;
            point.speed = speed_kmh;
            
            test_path_.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated straight line: length=%.1fm, %zu points",
                    length, test_path_.size());
    }
    
    void findClosestPoint()
    {
        double min_dist = std::numeric_limits<double>::max();
        size_t search_start = (path_index_ > 5) ? path_index_ - 5 : 0;
        size_t search_end = std::min(path_index_ + 30, test_path_.size());
        
        for (size_t i = search_start; i < search_end; ++i) {
            double dx = test_path_[i].x - current_x_;
            double dy = test_path_[i].y - current_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                path_index_ = i;
            }
        }
    }
    
    double calculateCrossTrackError()
    {
        if (path_index_ >= test_path_.size()) return 0.0;
        
        const auto& closest = test_path_[path_index_];
        double dx = current_x_ - closest.x;
        double dy = current_y_ - closest.y;
        
        return -dx * std::sin(closest.heading) + dy * std::cos(closest.heading);
    }
    
    double calculateHeadingError()
    {
        if (path_index_ >= test_path_.size()) return 0.0;
        
        double error = test_path_[path_index_].heading - current_heading_;
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;
        return error;
    }
    
    void publishTrajectory()
    {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& point : test_path_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            
            pose.pose.orientation.z = std::sin(point.heading / 2.0);
            pose.pose.orientation.w = std::cos(point.heading / 2.0);
            
            path_msg.poses.push_back(pose);
        }
        
        trajectory_pub_->publish(path_msg);
    }
    
    void publishStatus(const std::string& status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }
    
    void stopManeuver()
    {
        maneuver_active_ = false;
        
        auto active_msg = std_msgs::msg::Bool();
        active_msg.data = false;
        active_pub_->publish(active_msg);
        
        // Send zero commands
        auto steering_msg = std_msgs::msg::Float64();
        auto speed_msg = std_msgs::msg::Float64();
        steering_msg.data = 0.0;
        speed_msg.data = 0.0;
        steering_pub_->publish(steering_msg);
        speed_pub_->publish(speed_msg);
        
        publishStatus("Maneuver stopped");
        RCLCPP_INFO(this->get_logger(), "Test maneuver stopped");
    }
    
    struct TestPoint {
        double x;
        double y;
        double heading;
        double speed;
    };
    
    // ROS2 communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr maneuver_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State
    std::vector<TestPoint> test_path_;
    bool maneuver_active_;
    size_t path_index_;
    
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_heading_ = 0.0;
    double current_speed_ = 0.0;
    
    // Parameters
    double wheelbase_;
    double max_steering_angle_;
    double max_steering_wheel_angle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestManeuverNode>());
    rclcpp::shutdown();
    return 0;
}
