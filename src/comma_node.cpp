#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <car_control/msg/vehicle_state.hpp>
#include <chrono>
#include <memory>
#include <cstring>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <iostream>

using json = nlohmann::json;

class CommaNode : public rclcpp::Node
{
public:
    CommaNode() : Node("comma_node"), socket_fd_(-1), running_(true), listener_fd_(-1)
    {
        // Declare parameters
        this->declare_parameter("use_tcp_tunnel", false);
        this->declare_parameter("tcp_listen_port", 5555);
        this->declare_parameter("adb_host", "127.0.0.1");
        this->declare_parameter("adb_port", 5555);
        this->declare_parameter("reconnect_interval_sec", 5.0);
        
        // Subscriber for control commands
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&CommaNode::cmd_callback, this, std::placeholders::_1));
        
        speed_publisher_ = this->create_publisher<car_control::msg::VehicleState>(
            "vehicle/state", 10);
        
        // Initialize control state
        current_acceleration_ = 0.0;
        current_steering_ = 0.0;
        send_sequence_ = 0;
        
        // Initialize sensor state
        ever_received_ = false;
        latest_state_.timestamp = 0;           // int64 nanoseconds
        latest_state_.v_ego = 0.0f;           // float32
        latest_state_.steering_angle_deg = 0.0f;  // float32
        latest_state_.rear_wheel_speed_left = 0.0f;    // float32
        latest_state_.rear_wheel_speed_right = 0.0f;   // float32
        latest_state_.steering_torque = 0.0f;     // float32
        latest_state_.actuators_accel = 0.0f;     // float32
        latest_state_.actuators_torque = 0.0f;    // float32
        latest_state_.car_output_accel = 0.0f;    // float32
        latest_state_.car_output_torque = 0.0f;   // float32
        latest_state_.lat_active = false;         // bool
        latest_state_.long_active = false;        // bool
        
        // Timer-based publisher at 50 Hz for smooth, jitter-free output
        // Decouples receiving (minimal latency) from publishing (smooth rate)
        publish_timer_ = this->create_wall_timer(
            std::chrono::microseconds(20000),  // 50 Hz = 20ms = 20000us
            std::bind(&CommaNode::timer_publish_callback, this));
        
        // Start separate send and receive threads
        sender_thread_ = std::thread(&CommaNode::adb_sender_loop, this);
        reader_thread_ = std::thread(&CommaNode::adb_reader_loop, this);
        
        bool use_tcp_tunnel = this->get_parameter("use_tcp_tunnel").as_bool();
        if (use_tcp_tunnel) {
            RCLCPP_INFO(this->get_logger(), "Comma TCP Tunnel Node initialized (listening mode) with 50 Hz timer-based publishing");
        } else {
            RCLCPP_INFO(this->get_logger(), "Comma ADB Node initialized with 50 Hz timer-based publishing");
        }
    }
    
    ~CommaNode()
    {
        running_ = false;
        // Close listener socket if in TCP tunnel mode
        if (listener_fd_ >= 0) {
            close(listener_fd_);
            listener_fd_ = -1;
        }
        // Close data socket FIRST to unblock recv() in reader thread
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        // NOW threads can exit their loops
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
    }

private:
    void adb_sender_loop()
    {
        // Send control commands at 50 Hz
        const auto send_interval = std::chrono::milliseconds(20);
        
        while (running_) {
            auto start_time = std::chrono::steady_clock::now();
            
            if (socket_fd_ >= 0) {
                send_control_command();
            } else if (running_) {
                // Only sleep if still running
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            // Maintain precise 50 Hz timing
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto sleep_time = send_interval - elapsed;
            if (sleep_time > std::chrono::milliseconds(0) && running_) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    }
    
    void adb_reader_loop()
    {
        bool use_tcp_tunnel = this->get_parameter("use_tcp_tunnel").as_bool();
        
        if (use_tcp_tunnel) {
            tcp_listener_loop();
        } else {
            adb_connection_loop();
        }
    }
    
    void tcp_listener_loop()
    {
        // TCP tunnel listen mode - accept incoming connections
        int port = this->get_parameter("tcp_listen_port").as_int();
        
        // Create and bind listener socket
        listener_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listener_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create listener socket");
            return;
        }
        
        // Allow socket reuse
        int reuse = 1;
        setsockopt(listener_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        
        if (bind(listener_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind listener socket on port %d", port);
            close(listener_fd_);
            listener_fd_ = -1;
            return;
        }
        
        if (listen(listener_fd_, 5) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen on port %d", port);
            close(listener_fd_);
            listener_fd_ = -1;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "TCP tunnel listening on port %d", port);
        
        while (running_) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(listener_fd_, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) {
                if (running_) {
                    RCLCPP_WARN(this->get_logger(), "Accept failed");
                }
                continue;
            }
            
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), "Client connected from %s:%d", client_ip, ntohs(client_addr.sin_port));
            
            // Apply socket optimizations
            optimize_socket(client_fd);
            
            // Use this client socket for communication
            socket_fd_ = client_fd;
            
            // Read messages until connection closes
            while (running_ && socket_fd_ == client_fd) {
                read_adb_messages();
            }
            
            // Connection closed, wait for next client
            if (socket_fd_ == client_fd) {
                socket_fd_ = -1;
            }
        }
        
        if (listener_fd_ >= 0) {
            close(listener_fd_);
            listener_fd_ = -1;
        }
    }
    
    void adb_connection_loop()
    {
        // Blocking receive loop - no polling
        while (running_) {
            if (socket_fd_ < 0) {
                // Don't reconnect if shutting down
                if (!running_) break;
                
                try_connect();
                if (socket_fd_ < 0) {
                    // Connection attempt failed - wait before retrying
                    double interval = this->get_parameter("reconnect_interval_sec").as_double();
                    for (int i = 0; i < static_cast<int>(interval * 10) && running_; i++) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    continue;
                }
            }
            
            // Block on recv() - only publish when data actually arrives
            read_adb_messages();

            // If the connection dropped (recv returned 0 or error), wait before
            // reconnecting. Without this, when the ADB server dies but the local
            // ADB daemon still accepts the TCP port, connect() succeeds instantly
            // and we spin in a tight loop reconnecting hundreds of times per second.
            if (socket_fd_ < 0 && running_) {
                double interval = this->get_parameter("reconnect_interval_sec").as_double();
                RCLCPP_INFO(this->get_logger(), "Waiting %.1fs before reconnecting...", interval);
                for (int i = 0; i < static_cast<int>(interval * 10) && running_; i++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
    }
    
    void optimize_socket(int fd)
    {
        // ===== LOW LATENCY SOCKET OPTIMIZATIONS =====
        
        // 1. Disable Nagle's algorithm - send small packets immediately
        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        
        // 2. Reduce socket buffer sizes to minimize buffering delay
        int small_buffer = 8192;  // 8KB instead of default ~200KB
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &small_buffer, sizeof(small_buffer));
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &small_buffer, sizeof(small_buffer));
        
        // 3. Set socket priority for real-time traffic (requires CAP_NET_ADMIN or root)
        int priority = 6;  // High priority (0-7 scale)
        setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
    }
    
    void try_connect()
    {
        std::string host = this->get_parameter("adb_host").as_string();
        int port = this->get_parameter("adb_port").as_int();
        
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to create socket");
            return;
        }
        
        optimize_socket(socket_fd_);
        
        // BLOCKING recv - no timeout, wait for data
        // Reader thread processes messages immediately (minimal latency)
        // Timer publishes at steady 100Hz (no jitter)
        
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        
        if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid ADB host: %s", host.c_str());
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
        
        if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Connection to %s:%d failed - retrying...", host.c_str(), port);
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to comma device at %s:%d", 
            host.c_str(), port);
    }
    
    void read_adb_messages()
    {
        uint8_t buffer[4096];
        // BLOCKING recv - waits for data to arrive (no polling)
        int bytes_read = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
        
        if (bytes_read < 0) {
            RCLCPP_WARN(this->get_logger(), "Socket read error - reconnecting...");
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
        
        if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "Connection closed - reconnecting...");
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
        
        buffer[bytes_read] = '\0';
        receive_buffer_ += std::string((char*)buffer, bytes_read);
        
        // Parse complete JSON messages (newline-delimited)
        size_t pos = 0;
        int messages_in_buffer = 0;
        while ((pos = receive_buffer_.find('\n')) != std::string::npos) {
            std::string line = receive_buffer_.substr(0, pos);
            receive_buffer_.erase(0, pos + 1);
            
            if (line.empty() || line[0] != '{') {
                RCLCPP_DEBUG(this->get_logger(), "Skipping non-JSON line: %s", line.c_str());
                continue;
            }
            
            messages_in_buffer++;
            
            RCLCPP_DEBUG(this->get_logger(), "Raw JSON (first 120 chars): %.120s", line.c_str());
            
            try {
                json msg = json::parse(line);
                process_sensor_message(msg);
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Failed to parse JSON: %s | Line: %s", e.what(), line.c_str());
            }
        }
        
        // Log when multiple messages arrive in one recv()
        if (messages_in_buffer > 1) {
            RCLCPP_DEBUG(this->get_logger(),
                "BURST: Received %d messages in one recv() call (%d bytes) - timer will smooth output", 
                messages_in_buffer, bytes_read);
        }
    }
    
    void process_sensor_message(const json& msg)
    {
        std::string msg_type = msg.value("type", "");
        
        if (msg_type == "sensor") {
            // Process IMMEDIATELY for minimal latency - just store latest values
            // Timer will publish at steady 100Hz
            RCLCPP_DEBUG(this->get_logger(), 
                "Sensor message - has wheelSpeeds_rl: %d, has wheelSpeeds_rr: %d, has vEgo: %d, has steeringAngleDeg: %d",
                msg.contains("wheelSpeeds_rl"), msg.contains("wheelSpeeds_rr"),
                msg.contains("vEgo"), msg.contains("steeringAngleDeg"));
            
            // Debug: Print actual JSON values
            if (msg.contains("wheelSpeeds_rl")) {
                RCLCPP_DEBUG(this->get_logger(), "wheelSpeeds_rl type: %s, is_null: %d, value: %s",
                    msg["wheelSpeeds_rl"].type_name(), msg["wheelSpeeds_rl"].is_null(),
                    msg["wheelSpeeds_rl"].dump().c_str());
            }
            if (msg.contains("vEgo")) {
                RCLCPP_DEBUG(this->get_logger(), "vEgo type: %s, is_null: %d, value: %s",
                    msg["vEgo"].type_name(), msg["vEgo"].is_null(),
                    msg["vEgo"].dump().c_str());
            }
            if (msg.contains("steeringAngleDeg")) {
                RCLCPP_DEBUG(this->get_logger(), "steeringAngleDeg type: %s, is_null: %d, value: %s",
                    msg["steeringAngleDeg"].type_name(), msg["steeringAngleDeg"].is_null(),
                    msg["steeringAngleDeg"].dump().c_str());
            }
            
            bool has_wheel_speeds = msg.contains("wheelSpeeds_rl") && msg.contains("wheelSpeeds_rr") &&
                                   !msg["wheelSpeeds_rl"].is_null() && !msg["wheelSpeeds_rr"].is_null();
            bool has_speed = msg.contains("vEgo") && !msg["vEgo"].is_null();
            bool has_steering = msg.contains("steeringAngleDeg") && !msg["steeringAngleDeg"].is_null();
            bool has_timestamp = msg.contains("timestamp") && !msg["timestamp"].is_null();
            bool has_steering_torque = msg.contains("steeringTorque") && !msg["steeringTorque"].is_null();
            bool has_actuators_accel = msg.contains("actuators_accel") && !msg["actuators_accel"].is_null();
            bool has_actuators_torque = msg.contains("actuators_torque") && !msg["actuators_torque"].is_null();
            bool has_car_output_accel = msg.contains("carOutput_accel") && !msg["carOutput_accel"].is_null();
            bool has_car_output_torque = msg.contains("carOutput_torque") && !msg["carOutput_torque"].is_null();
            bool has_lat_active = msg.contains("lat_active") && !msg["lat_active"].is_null();
            bool has_long_active = msg.contains("long_active") && !msg["long_active"].is_null();
            
            RCLCPP_DEBUG(this->get_logger(),
                "Field availability - Wheels: %d, Speed: %d, Steering: %d | Values - vEgo: %.2f, steering: %.2f",
                has_wheel_speeds, has_speed, has_steering,
                has_speed ? msg["vEgo"].get<double>() : -999.0,
                has_steering ? msg["steeringAngleDeg"].get<double>() : -999.0);
            
            // Store latest sensor data (minimal latency - no publishing here)
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            
            bool updated = false;
            
            if (has_timestamp) {
                latest_state_.timestamp = msg["timestamp"].get<int64_t>();
                updated = true;
            }
            
            if (has_speed) {
                latest_state_.v_ego = msg["vEgo"].get<float>() * 3.6f;  // Convert m/s to km/h
                updated = true;
            }
            
            if (has_steering) {
                latest_state_.steering_angle_deg = msg["steeringAngleDeg"].get<float>();
                updated = true;
            }
            
            if (has_wheel_speeds) {
                latest_state_.rear_wheel_speed_left = msg["wheelSpeeds_rl"].get<float>();
                latest_state_.rear_wheel_speed_right = msg["wheelSpeeds_rr"].get<float>();
                updated = true;
            }
            
            if (has_steering_torque) {
                latest_state_.steering_torque = msg["steeringTorque"].get<float>();
                updated = true;
            }
            
            if (has_actuators_accel) {
                latest_state_.actuators_accel = msg["actuators_accel"].get<float>();
                updated = true;
            }
            
            if (has_actuators_torque) {
                latest_state_.actuators_torque = msg["actuators_torque"].get<float>();
                updated = true;
            }
            
            if (has_car_output_accel) {
                latest_state_.car_output_accel = msg["carOutput_accel"].get<float>();
                updated = true;
            }
            
            if (has_car_output_torque) {
                latest_state_.car_output_torque = msg["carOutput_torque"].get<float>();
                updated = true;
            }
            
            if (has_lat_active) {
                latest_state_.lat_active = msg["lat_active"].get<bool>();
                updated = true;
            }
            
            if (has_long_active) {
                latest_state_.long_active = msg["long_active"].get<bool>();
                updated = true;
            }
            
            if (updated) {
                ever_received_ = true;
            }
            
        } else if (msg_type == "pong") {
            RCLCPP_DEBUG(this->get_logger(), "Pong received");
        }
    }
    
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Store control commands from path follower or planner
        // msg->linear.x = acceleration (-1 to 1, where 1 = full throttle, -1 = full brake)
        // msg->angular.z = steering torque (-1 to 1)
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // Clamp values to [-1, 1] range
        current_acceleration_ = std::max(-1.0, std::min(1.0, msg->linear.x));
        current_steering_ = std::max(-1.0, std::min(1.0, msg->angular.z));
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Control command - Accel: %.3f, Steer: %.3f", 
            current_acceleration_, current_steering_);
    }
    
    void send_control_command()
    {
        if (socket_fd_ < 0) return;
        
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // Build joystick command JSON
        // axes[0] = acceleration (gas/brake, -1 to 1)
        // axes[1] = steering_torque (steering wheel angle, -1 to 1)
        json cmd;
        cmd["type"] = "joystick";
        cmd["axes"] = json::array({current_acceleration_, current_steering_});
        cmd["loggingEnabled"] = false;
        cmd["time"] = rclcpp::Time().nanoseconds() / 1e9;
        cmd["seq"] = send_sequence_++;
        
        std::string cmd_str = cmd.dump() + "\n";
        
        if (send(socket_fd_, cmd_str.c_str(), cmd_str.length(), MSG_NOSIGNAL) < 0) {
            if (errno != EPIPE) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Failed to send control command");
            }
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }
    
    // Timer callback - publishes at 50 Hz for smooth, jitter-free output
    void timer_publish_callback()
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        
        if (!ever_received_) {
            // Haven't received any data from comma yet - don't publish zeros
            return;
        }
        
        // Always publish latest state at steady 50 Hz (even if data unchanged since last tick)
        // This keeps the dashboard Hz counter accurate and matches behaviour of all other nodes
        speed_publisher_->publish(latest_state_);
        
        RCLCPP_DEBUG(this->get_logger(),
            "Published state: speed=%.2f km/h, steering=%.2f deg, ts=%ld ns",
            latest_state_.v_ego, latest_state_.steering_angle_deg, latest_state_.timestamp);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
    rclcpp::Publisher<car_control::msg::VehicleState>::SharedPtr speed_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    std::thread sender_thread_;
    std::thread reader_thread_;
    int socket_fd_;
    int listener_fd_;  // For TCP tunnel listen mode
    std::atomic<bool> running_;
    
    // Control state (for sending commands)
    std::mutex cmd_mutex_;
    double current_acceleration_;  // -1 to 1
    double current_steering_;      // -1 to 1
    uint64_t send_sequence_;
    
    // Latest sensor state (received data, published by timer)
    std::mutex sensor_mutex_;
    bool ever_received_;  // true once first sensor message parsed; prevents publishing zero-state
    car_control::msg::VehicleState latest_state_;
    
    // Receive buffer for incomplete messages
    std::string receive_buffer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CommaNode>();
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    

    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}