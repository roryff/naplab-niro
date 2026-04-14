#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "car_control/msg/vehicle_state.hpp"
#include "car_control/msg/esf_status.hpp"
#include "car_control/msg/esf_sensor.hpp"
#include <chrono>
#include <memory>
#include <cmath>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/tcp.h>
#include "car_control/ubx_protocol.hpp"
#include "car_control/geo_utils.hpp"

/**
 * @brief GNSS Node for autonomous vehicle navigation
 * 
 * This node connects directly to a u-blox GNSS receiver via TCP, parses UBX protocol
 * messages, and converts GPS coordinates to local ENU frame for path following.
 * Supports RTK and Automotive Dead Reckoning (ADR) modes.
 * 
 * **IMU Data Availability:**
 * - ESF-STATUS (Class 0x10, ID 0x10): IMU calibration/fusion status - only in ADR mode
 * - ESF-ALG (Class 0x10, ID 0x14): IMU alignment status - only in ADR mode
 * - NAV-PVT (Class 0x01, ID 0x07): Contains headVeh from heading computation, always available
 * 
 * If you see NO ESF-* messages but NAV-PVT messages: ADR mode is disabled/not available
 * If you see ESF-* with fusionMode=0 (INIT): IMU is calibrating (requires motion)
 * If you see ESF-* with fusionMode=1 (FUSION): IMU calibration complete, ADR active
 * 
 * Topics Published:
 *     - gnss/pose (geometry_msgs/PoseStamped): Local ENU position
 *     - gnss/fix (sensor_msgs/NavSatFix): GPS fix data
 *     - gnss/velocity (geometry_msgs/TwistStamped): Vehicle velocity
 *     - gnss/odometry (nav_msgs/Odometry): Combined pose and velocity
 *     - gnss/gyro (geometry_msgs/Vector3Stamped): Compensated angular rates x/y/z [rad/s] (ESF-INS)
 *     - gnss/accel (geometry_msgs/Vector3Stamped): Compensated accelerations x/y/z [m/s²] (ESF-INS)
 */
class GNSSNode : public rclcpp::Node
{
public:
    GNSSNode() : Node("gnss_node"), running_(true), socket_fd_(-1), origin_set_(false),
                 fusion_mode_(0xFF), alignment_status_(0), calibration_complete_(false),
                 fusion_disabled_warned_(false)
    {
        // Declare parameters
        this->declare_parameter("host", "tppg2.lan");
        this->declare_parameter("port", 7799);
        this->declare_parameter("reconnect_interval_sec", 5.0);
        this->declare_parameter("auto_set_origin", true);
        this->declare_parameter("origin_lat", 0.0);
        this->declare_parameter("origin_lon", 0.0);
        this->declare_parameter("origin_alt", 0.0);
        
        // Publishers
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "gnss/pose", 10);
        navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "gnss/fix", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "gnss/velocity", 10);
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "gnss/odometry", 10);
        esf_status_pub_ = this->create_publisher<car_control::msg::EsfStatus>(
            "gnss/esf_status", 10);
        gyro_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "gnss/gyro", 10);
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "gnss/accel", 10);
        
        // Check if origin should be manually set
        if (!this->get_parameter("auto_set_origin").as_bool()) {
            origin_lat_ = this->get_parameter("origin_lat").as_double();
            origin_lon_ = this->get_parameter("origin_lon").as_double();
            origin_alt_ = this->get_parameter("origin_alt").as_double();
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), 
                "Manual origin set: lat=%.7f, lon=%.7f, alt=%.2f",
                origin_lat_, origin_lon_, origin_alt_);
        }
        
        // Subscribe to vehicle wheel speed data to feed odometer input to u-blox.
        // Callback only stores the latest speed; a dedicated sender thread sends
        // at a fixed 50 Hz so the receiver sees isochronous data (needed to auto-
        // detect the sampling rate for scale factor estimation).
        vehicle_state_sub_ = this->create_subscription<car_control::msg::VehicleState>(
            "/vehicle/state", 10,
            std::bind(&GNSSNode::vehicle_state_callback, this, std::placeholders::_1));

        // Start sender thread (fixed 50 Hz ESF-MEAS output)
        sender_thread_ = std::thread(&GNSSNode::sender_loop, this);

        // Start reader thread (blocks on socket, publishes immediately on data arrival)
        reader_thread_ = std::thread(&GNSSNode::reader_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "GNSS Node initialized - ADR mode only");
    }
    
    ~GNSSNode()
    {
        running_ = false;
        {
            // shutdown() immediately unblocks recv() in the reader thread.
            // close() alone is not guaranteed to do so on Linux.
            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            if (socket_fd_ >= 0) {
                ::shutdown(socket_fd_, SHUT_RDWR);
                close(socket_fd_);
                socket_fd_ = -1;
            }
        }
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
    }

private:
    /**
     * @brief Main reader loop - runs in dedicated thread
     * Blocks on socket recv() and publishes immediately when data arrives
     */
    void reader_loop()
    {
        while (running_) {
            if (socket_fd_ < 0) {
                if (!running_) break;
                try_connect();
                if (socket_fd_ < 0) {
                    // Sleep in small increments so SIGINT is handled promptly
                    for (int i = 0; i < 10 && running_; i++) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    continue;
                }
            
            // On fresh connection, enable ESF-INS output and disable ESF-RAW
            enable_esf_ins_output();
        }
            
        // Block on socket read - will return immediately when data arrives
            read_gnss_data();

            // If read_gnss_data() closed the socket (error / remote close),
            // wait before reconnecting.  Rapid SYN storms can exhaust the
            // u-blox's tiny TCP connection table and crash its NIC driver,
            // making the device completely unreachable until the cable is
            // replugged.
            if (socket_fd_ < 0 && running_) {
                for (int i = 0; i < 20 && running_; i++) {  // ~2 s back-off
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
    }
    
    /**
     * @brief Attempt to connect to u-blox receiver
     *
     * Uses a local fd throughout so that socket_fd_ (visible to the sender
     * thread) is only set once the socket is fully connected, configured, and
     * back in blocking mode.  This prevents the sender from calling send() on
     * a non-blocking or not-yet-connected fd (which would return EAGAIN).
     */
    void try_connect()
    {
        std::string host = this->get_parameter("host").as_string();
        int port = this->get_parameter("port").as_int();

        // Work with a local fd until the connection is fully established.
        // socket_fd_ is only written at the very end, under socket_write_mutex_,
        // so the sender thread never sees a half-ready descriptor.
        int new_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (new_fd < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to create socket: %s", strerror(errno));
            return;
        }

        // Set recv timeout – fallback so reader_loop can notice shutdown
        // even if shutdown(SHUT_RDWR) somehow doesn't unblock recv immediately.
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(new_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // TCP keepalive: keeps the connection entry alive in the switch/router
        // ARP and MAC tables, and detects a dead peer without waiting for a
        // send to fail.  Without this some switches age out the entry after
        // ~30 s of inactivity and then isolate the port, making the u-blox
        // completely unreachable until the cable is replugged.
        int keepalive = 1;
        setsockopt(new_fd, SOL_SOCKET,  SO_KEEPALIVE,  &keepalive, sizeof(keepalive));
        int keepidle  = 10;  // start probing after 10 s idle
        int keepintvl =  5;  // probe every 5 s
        int keepcnt   =  3;  // drop after 3 consecutive failures (~25 s total)
        setsockopt(new_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
        setsockopt(new_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
        setsockopt(new_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));

        // Disable Nagle: we send many small 50 Hz ESF-MEAS frames that must
        // not be held back by the coalescing algorithm.
        int nodelay = 1;
        setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        // Use non-blocking connect with a short timeout so SIGINT isn't blocked
        // by the OS TCP timeout (which can be ~2 minutes).
        // NOTE: new_fd is NOT published to socket_fd_ yet, so the sender thread
        // cannot touch it while it is in O_NONBLOCK mode.
        fcntl(new_fd, F_SETFL, O_NONBLOCK);

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);

        // Resolve hostname or parse IPv4 literal.
        if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
            struct addrinfo hints;
            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_STREAM;

            struct addrinfo* result = nullptr;
            int gai = getaddrinfo(host.c_str(), nullptr, &hints, &result);
            if (gai != 0 || result == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "Invalid address: %s", host.c_str());
                if (result) freeaddrinfo(result);
                close(new_fd);
                return;
            }

            auto* addr_in = reinterpret_cast<struct sockaddr_in*>(result->ai_addr);
            server_addr.sin_addr = addr_in->sin_addr;
            freeaddrinfo(result);
        }

        int ret = connect(new_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (ret < 0 && errno != EINPROGRESS) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Connection to %s:%d failed: %s - retrying...",
                host.c_str(), port, strerror(errno));
            close(new_fd);
            return;
        }

        if (ret != 0) {
            // Wait up to 5 seconds for connect, but bail immediately if shutting down
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(new_fd, &write_fds);
            struct timeval connect_tv = {5, 0};
            int sel = select(new_fd + 1, nullptr, &write_fds, nullptr, &connect_tv);

            if (!running_) {
                close(new_fd);
                return;
            }

            if (sel <= 0) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Connection to %s:%d timed out - retrying...", host.c_str(), port);
                close(new_fd);
                return;
            }

            // Check if connect actually succeeded
            int sock_err = 0;
            socklen_t sock_err_len = sizeof(sock_err);
            getsockopt(new_fd, SOL_SOCKET, SO_ERROR, &sock_err, &sock_err_len);
            if (sock_err != 0) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Connection to %s:%d failed: %s - retrying...",
                    host.c_str(), port, strerror(sock_err));
                close(new_fd);
                return;
            }
        }

        // Restore blocking mode BEFORE handing the fd to the sender thread.
        fcntl(new_fd, F_SETFL, fcntl(new_fd, F_GETFL) & ~O_NONBLOCK);

        // Publish the ready fd atomically so the sender thread sees a fully
        // connected, blocking socket or nothing at all.
        {
            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            socket_fd_ = new_fd;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to u-blox receiver at %s:%d",
            host.c_str(), port);
    }
    
    /**
     * @brief Read data from u-blox receiver (blocking)
     * This runs in a dedicated thread and publishes immediately when data arrives
     */
    void read_gnss_data()
    {
        // Take a snapshot of the fd so we recv() without holding the mutex
        // (recv blocks; holding the mutex would deadlock the sender thread).
        int fd;
        {
            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            fd = socket_fd_;
        }
        if (fd < 0) return;

        uint8_t buffer[2048];
        int bytes_read = recv(fd, buffer, sizeof(buffer), 0);

        if (bytes_read < 0) {
            // Timeout is OK - just means no data yet, loop will continue
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            RCLCPP_ERROR(this->get_logger(), "Socket read error: %s - reconnecting...",
                strerror(errno));
            // Close under the mutex so the sender never uses a dead fd.
            {
                std::lock_guard<std::mutex> lock(socket_write_mutex_);
                if (socket_fd_ == fd) {   // guard against concurrent reconnect
                    close(socket_fd_);
                    socket_fd_ = -1;
                }
            }
            rx_buffer_.clear();
            return;
        }

        if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "Connection closed - reconnecting...");
            {
                std::lock_guard<std::mutex> lock(socket_write_mutex_);
                if (socket_fd_ == fd) {
                    close(socket_fd_);
                    socket_fd_ = -1;
                }
            }
            rx_buffer_.clear();
            return;
        }

        // Add to buffer and parse UBX messages
        rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + bytes_read);
        parse_ubx_messages();
    }
    
    /**
     * @brief Parse UBX messages from receive buffer
     */
    void parse_ubx_messages()
    {
        while (rx_buffer_.size() >= sizeof(UBXHeader)) {
            // Look for UBX sync bytes (0xB5 0x62)
            auto sync_pos = rx_buffer_.begin();
            while (sync_pos != rx_buffer_.end() && *sync_pos != 0xB5) {
                ++sync_pos;
            }
            
            if (sync_pos == rx_buffer_.end()) {
                rx_buffer_.clear();
                return;
            }
            
            // Remove data before sync
            rx_buffer_.erase(rx_buffer_.begin(), sync_pos);
            
            if (rx_buffer_.size() < sizeof(UBXHeader)) {
                return;
            }
            
            // Check second sync byte
            if (rx_buffer_[1] != 0x62) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }
            
            // Parse header
            UBXHeader header;
            std::memcpy(&header, rx_buffer_.data(), sizeof(UBXHeader));
            
            size_t total_length = sizeof(UBXHeader) + header.length + 2; // +2 for checksum
            
            if (rx_buffer_.size() < total_length) {
                return; // Wait for more data
            }
            
            // Verify checksum
            if (!verify_checksum(rx_buffer_.data(), total_length)) {
                RCLCPP_WARN(this->get_logger(),
                    "UBX checksum failed for msg 0x%02X:0x%02X (len=%u)", 
                    header.msg_class, header.msg_id, header.length);
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }
            
            // Process NAV-PVT message (Class 0x01, ID 0x07)
            if (header.msg_class == 0x01 && header.msg_id == 0x07) {
                if (header.length == sizeof(UBXNAVPVT)) {
                    process_nav_pvt(rx_buffer_.data() + sizeof(UBXHeader));
                }
            }
            // Process ESF-STATUS message (Class 0x10, ID 0x10)
            else if (header.msg_class == 0x10 && header.msg_id == 0x10) {
                if (header.length >= sizeof(UBXESFSTATUS)) {
                    process_esf_status(rx_buffer_.data() + sizeof(UBXHeader));
                }
            }
            // Process ESF-ALG message (Class 0x10, ID 0x14)
            else if (header.msg_class == 0x10 && header.msg_id == 0x14) {
                if (header.length >= sizeof(UBXESFALG)) {
                    process_esf_alg(rx_buffer_.data() + sizeof(UBXHeader));
                }
            }
            // Process ESF-INS message (Class 0x10, ID 0x15) — compensated vehicle-frame dynamics
            else if (header.msg_class == 0x10 && header.msg_id == 0x15) {
                if (header.length >= sizeof(UBXESFINS)) {
                    process_esf_ins(rx_buffer_.data() + sizeof(UBXHeader));
                }
            }
            else {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Skipping UBX message 0x%02X:0x%02X (len=%u)",
                    header.msg_class, header.msg_id, header.length);
            }
            
            // Remove processed message
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_length);
        }
    }
    
    /** Verify UBX Fletcher-8 checksum. Delegates to ubx_protocol.hpp. */
    bool verify_checksum(const uint8_t* data, size_t length)
    {
        return ubx_verify_checksum(data, length);
    }
    
    /**
     * @brief Process NAV-PVT message
     */
    void process_nav_pvt(const uint8_t* payload)
    {
        UBXNAVPVT pvt;
        std::memcpy(&pvt, payload, sizeof(UBXNAVPVT));

        uint8_t carrSoln = (pvt.flags >> 6) & 0x03;
        
        auto timestamp = this->now();
        
        // Publish NavSatFix
        auto navsat_msg = sensor_msgs::msg::NavSatFix();
        navsat_msg.header.stamp = timestamp;
        navsat_msg.header.frame_id = "gps";
        
        navsat_msg.latitude = pvt.lat * 1e-7;
        navsat_msg.longitude = pvt.lon * 1e-7;
        navsat_msg.altitude = pvt.hMSL * 1e-3; // Convert mm to meters
        
        // Set fix status based on u-blox fixType
        // 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
        if (pvt.fixType >= 3) {
            // Check for RTK fix in flags
            if (carrSoln == 2) {
                navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; // RTK Fixed
            } else if (carrSoln == 1) {
                navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX; // RTK Float
            } else {
                navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // 3D fix
            }
        } else if (pvt.fixType == 2) {
            navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // 2D fix
        } else if (pvt.fixType == 1 || pvt.fixType == 4) {
            navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; // Dead reckoning
        } else {
            navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        
        navsat_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        // Set covariance from accuracy estimates
        double h_acc = pvt.hAcc * 1e-3; // mm to meters
        double v_acc = pvt.vAcc * 1e-3;
        navsat_msg.position_covariance[0] = h_acc * h_acc;
        navsat_msg.position_covariance[4] = h_acc * h_acc;
        navsat_msg.position_covariance[8] = v_acc * v_acc;
        navsat_msg.position_covariance_type = 
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        navsat_publisher_->publish(navsat_msg);
        
        // Convert lat/lon to UTM32 (EPSG:25832) absolute coordinates
        if (!origin_set_) {
            origin_alt_ = navsat_msg.altitude;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Altitude origin set to first fix: alt=%.2f m",
                origin_alt_);
        }

        double east, north;
        geo::latlon_to_utm32(navsat_msg.latitude, navsat_msg.longitude, east, north);
        
        // Publish pose in local frame
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header = navsat_msg.header;
        pose_msg.header.frame_id = "utm32"; // UTM zone 32N (EPSG:25832)
        pose_msg.pose.position.x = east;
        pose_msg.pose.position.y = north;
        pose_msg.pose.position.z = navsat_msg.altitude - origin_alt_;
        
        // headVeh: NED heading (CW from North), in 1e-5 deg. Valid when flags2 bit 5 set.
        // IMU/ADR sensor fusion keeps this valid continuously even without GNSS fix.
        // ENU yaw (CCW from East) = π/2 − NED_heading.
        if ((pvt.flags2 >> 5) & 0x01) {
            last_valid_enu_yaw_ = M_PI / 2.0 - pvt.headVeh * 1e-5 * M_PI / 180.0;
        }
        // else: hold last valid heading (fusion momentarily unavailable)
        double enu_yaw = last_valid_enu_yaw_;
        pose_msg.pose.orientation.w = std::cos(enu_yaw / 2.0);
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(enu_yaw / 2.0);

        pose_publisher_->publish(pose_msg);
        auto velocity_msg = geometry_msgs::msg::TwistStamped();
        velocity_msg.header = navsat_msg.header;
        velocity_msg.twist.linear.x = pvt.velN * 1e-3; // mm/s to m/s (North)
        velocity_msg.twist.linear.y = pvt.velE * 1e-3; // mm/s to m/s (East)
        velocity_msg.twist.linear.z = -pvt.velD * 1e-3; // mm/s to m/s (Up, note sign flip)
        
        velocity_publisher_->publish(velocity_msg);
        
        // Publish combined odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header = navsat_msg.header;
        odom_msg.header.frame_id = "utm32";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist = velocity_msg.twist;
        
        odometry_publisher_->publish(odom_msg);
        
        // Log status with fix type details
        std::string fix_status;
        if (pvt.fixType == 0) {
            fix_status = "NO FIX";
        } else if (pvt.fixType == 1) {
            fix_status = "DEAD RECKONING";
        } else if (pvt.fixType == 4) {
            fix_status = "GNSS+DEAD RECKONING";
        } else if (carrSoln == 2) {
            fix_status = "RTK FIXED";
        } else if (carrSoln == 1) {
            fix_status = "RTK FLOAT";
        } else if (pvt.fixType == 3) {
            fix_status = "3D FIX";
        } else if (pvt.fixType == 2) {
            fix_status = "2D FIX";
        } else {
            fix_status = "UNKNOWN";
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GNSS: %s | sats=%d | UTM32: E=%.2f N=%.2f z=%.2f | vN=%.2f vE=%.2f vD=%.2f m/s | calibrated=%s",
            fix_status.c_str(), pvt.numSV,
            east, north, navsat_msg.altitude - origin_alt_,
            pvt.velN * 1e-3, pvt.velE * 1e-3, pvt.velD * 1e-3,
            calibration_complete_ ? "YES" : "NO");
    }
    
    /** Convert lat/lon to UTM32 (EPSG:25832). Delegates to geo_utils.hpp. */
    void latlon_to_enu(double lat, double lon, double& east, double& north)
    {
        geo::latlon_to_utm32(lat, lon, east, north);
    }
    
    /**
     * @brief Process ESF-STATUS message (IMU calibration status)
     */
    void process_esf_status(const uint8_t* payload)
    {
        UBXESFSTATUS status;
        std::memcpy(&status, payload, sizeof(UBXESFSTATUS));

        const char* fusion_str[] = {"INIT", "FUSION", "SUSPENDED", "DISABLED"};
        const char* fusion_mode_str = (status.fusionMode < 4) ?
            fusion_str[status.fusionMode] : "UNKNOWN";

        fusion_mode_ = status.fusionMode;

        // Calibration state transitions
        bool was_calibrated = calibration_complete_;
        calibration_complete_ = (status.fusionMode == 1);
        if (calibration_complete_ && !was_calibrated) {
            RCLCPP_INFO(this->get_logger(), "IMU calibration complete - fusion mode active");
        } else if (!calibration_complete_ && was_calibrated) {
            RCLCPP_WARN(this->get_logger(),
                "IMU calibration lost - fusion mode changed to %s", fusion_mode_str);
        }
        if (status.fusionMode == 3 && !fusion_disabled_warned_) {
            RCLCPP_WARN(this->get_logger(),
                "Fusion mode is DISABLED. ADR is not active. Ensure ESF-MEAS input is configured.");
            fusion_disabled_warned_ = true;
        } else if (status.fusionMode != 3) {
            fusion_disabled_warned_ = false;
        }

        // --- Decode initStatus1 ---
        const char* two_bit_str[] = {"off", "initializing", "initialized", "initialized"};
        uint8_t wtInit  = (status.initStatus1 >> 0) & 0x03;
        uint8_t mntAlg  = (status.initStatus1 >> 2) & 0x03;
        uint8_t insInit = (status.initStatus1 >> 4) & 0x03;
        uint8_t imuInit = (status.initStatus2 >> 0) & 0x03;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "ESF-STATUS: fusion=%-9s | wt=%-12s mntAlg=%-12s ins=%-12s imu=%-12s | numSens=%u",
            fusion_mode_str,
            two_bit_str[wtInit], two_bit_str[mntAlg],
            two_bit_str[insInit], two_bit_str[imuInit],
            status.numSens);

        // --- Per-sensor status ---
        // Build entire sensor block into one string to avoid RCLCPP_DEBUG_THROTTLE
        // suppressing all but the first iteration (throttle key is per call-site).
        const char* calib_str[] = {"not-calibrated", "calibrating", "calibrated", "calibrated"};
        const char* time_str[]  = {"no-data", "byte-tag", "event-tag", "provided"};
        const uint8_t* sens_ptr = payload + sizeof(UBXESFSTATUS);
        std::string sens_log;
        for (uint8_t i = 0; i < status.numSens; i++) {
            UBXESFSensor s;
            std::memcpy(&s, sens_ptr + i * sizeof(UBXESFSensor), sizeof(UBXESFSensor));

            uint8_t s_type  = s.sensStatus1 & 0x3F;
            bool    s_used  = (s.sensStatus1 >> 6) & 0x01;
            bool    s_ready = (s.sensStatus1 >> 7) & 0x01;
            uint8_t s_calib = (s.sensStatus2 >> 0) & 0x03;
            uint8_t s_time  = (s.sensStatus2 >> 2) & 0x03;

            std::string faults;
            if (s.faults & 0x01) faults += "badMeas ";
            if (s.faults & 0x02) faults += "badTTag ";
            if (s.faults & 0x04) faults += "missingMeas ";
            if (s.faults & 0x08) faults += "noisyMeas";
            if (faults.empty()) faults = "none";

            char line[160];
            std::snprintf(line, sizeof(line),
                "\n  [%u] type=%-2u used=%s ready=%s calib=%-14s time=%-9s freq=%2uHz faults=[%s]",
                i, s_type,
                s_used  ? "Y" : "N",
                s_ready ? "Y" : "N",
                calib_str[s_calib], time_str[s_time],
                s.freq, faults.c_str());
            sens_log += line;
        }
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "ESF-STATUS sensors:%s", sens_log.c_str());

        // --- Publish typed EsfStatus message ---
        // Sensor type -> human name (ZED-F9R ADR sensor IDs)
        auto type_name = [](uint8_t t) -> const char* {
            switch (t) {
                case  5: return "Gyro Z";
                case 11: return "Wheel Speed";
                case 13: return "Gyro X";
                case 14: return "Gyro Y";
                case 16: return "Accel X";
                case 17: return "Accel Y";
                case 18: return "Accel Z";
                default: return "Unknown";
            }
        };

        car_control::msg::EsfStatus esf_pub_msg;
        esf_pub_msg.header.stamp = this->now();
        esf_pub_msg.fusion_mode   = status.fusionMode;
        esf_pub_msg.fusion_label  = fusion_mode_str;
        esf_pub_msg.wt_label      = two_bit_str[wtInit];
        esf_pub_msg.mnt_alg_label = two_bit_str[mntAlg];
        esf_pub_msg.ins_label     = two_bit_str[insInit];
        esf_pub_msg.imu_label     = two_bit_str[imuInit];
        esf_pub_msg.num_sens      = status.numSens;

        for (uint8_t i = 0; i < status.numSens; i++) {
            UBXESFSensor raw;
            std::memcpy(&raw, sens_ptr + i * sizeof(UBXESFSensor), sizeof(UBXESFSensor));
            uint8_t s_type  = raw.sensStatus1 & 0x3F;
            bool    s_used  = (raw.sensStatus1 >> 6) & 0x01;
            bool    s_ready = (raw.sensStatus1 >> 7) & 0x01;
            uint8_t s_calib = (raw.sensStatus2 >> 0) & 0x03;
            uint8_t s_time  = (raw.sensStatus2 >> 2) & 0x03;

            std::string faults;
            if (raw.faults & 0x01) faults += "badMeas ";
            if (raw.faults & 0x02) faults += "badTTag ";
            if (raw.faults & 0x04) faults += "missingMeas ";
            if (raw.faults & 0x08) faults += "noisyMeas";
            if (!faults.empty() && faults.back() == ' ') faults.pop_back();
            if (faults.empty()) faults = "none";

            car_control::msg::EsfSensor sens;
            sens.type      = s_type;
            sens.type_name = type_name(s_type);
            sens.used      = s_used;
            sens.ready     = s_ready;
            sens.calib     = calib_str[s_calib];
            sens.time_tag  = time_str[s_time];
            sens.freq      = raw.freq;
            sens.faults    = faults;
            esf_pub_msg.sensors.push_back(sens);
        }
        esf_status_pub_->publish(esf_pub_msg);
    }
    
    /**
     * @brief Process ESF-ALG message (IMU alignment status)
     */
    void process_esf_alg(const uint8_t* payload)
    {
        UBXESFALG alg;
        std::memcpy(&alg, payload, sizeof(UBXESFALG));
        
        // Decode alignment status from flags
        // Bit 0: autoMntAlgOn
        // Bit 1-2: status (0=user-defined, 1=roll/pitch, 2=roll/pitch/yaw, 3=fine)
        uint8_t status_bits = (alg.flags >> 1) & 0x03;
        uint8_t auto_on = alg.flags & 0x01;
        
        const char* align_status[] = {
            "USER-DEFINED",
            "COARSE (Roll/Pitch only)",
            "COARSE ALIGNED (Roll/Pitch/Yaw)",
            "FINE ALIGNED"
        };
        
        const char* status_str = (status_bits < 4) ? align_status[status_bits] : "UNKNOWN";
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "ESF-ALG: status=%s auto=%s yaw=%.4f pitch=%.4f roll=%.4f err=0x%02X",
            status_str, auto_on ? "ON" : "OFF",
            alg.yaw * 1e-2, alg.pitch * 1e-2, alg.roll * 1e-2, alg.error);
        
        uint8_t old_status = alignment_status_;
        alignment_status_ = alg.flags;
        
        // Log alignment status changes
        if (alignment_status_ != old_status) {
            RCLCPP_INFO(this->get_logger(), 
                "IMU alignment status: %s", status_str);
        }
    }


    /**
     * @brief Process ESF-INS message — publish compensated vehicle-frame dynamics
     *
     * Angular rates: int32 [deg/s * 1e-3] → rad/s (per-axis validity checked)
     * Accelerations: int32 [mg]           → m/s² (gravity-free, per-axis validity checked)
     *
     * NOTE: Fields are only meaningful when fusionMode == 1 (FUSION).
     * Publishes gnss/gyro [rad/s] and gnss/accel [m/s²].
     */
    void process_esf_ins(const uint8_t* payload)
    {
        UBXESFINS ins;
        std::memcpy(&ins, payload, sizeof(UBXESFINS));

        auto stamp = this->get_clock()->now();
        constexpr double DEG_S_SCALE = 1e-3 * M_PI / 180.0;  // 0.001 deg/s per LSB → rad/s
        constexpr double MG_SCALE    = 1e-3 * 9.80665;        // mg → m/s²

        // Angular rates
        if (ins.bitfield0 & (UBX_ESF_INS_X_ANG_RATE_VALID |
                             UBX_ESF_INS_Y_ANG_RATE_VALID |
                             UBX_ESF_INS_Z_ANG_RATE_VALID)) {
            geometry_msgs::msg::Vector3Stamped gyro_msg;
            gyro_msg.header.stamp    = stamp;
            gyro_msg.header.frame_id = "imu";
            gyro_msg.vector.x = (ins.bitfield0 & UBX_ESF_INS_X_ANG_RATE_VALID)
                                 ? ins.xAngRate * DEG_S_SCALE : 0.0;
            gyro_msg.vector.y = (ins.bitfield0 & UBX_ESF_INS_Y_ANG_RATE_VALID)
                                 ? ins.yAngRate * DEG_S_SCALE : 0.0;
            gyro_msg.vector.z = (ins.bitfield0 & UBX_ESF_INS_Z_ANG_RATE_VALID)
                                 ? ins.zAngRate * DEG_S_SCALE : 0.0;
            gyro_pub_->publish(gyro_msg);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "ESF-INS gyro [deg/s]: x=%.3f y=%.3f z=%.3f",
                ins.xAngRate * 1e-3, ins.yAngRate * 1e-3, ins.zAngRate * 1e-3);
        }

        // Accelerations
        if (ins.bitfield0 & (UBX_ESF_INS_X_ACCEL_VALID |
                             UBX_ESF_INS_Y_ACCEL_VALID |
                             UBX_ESF_INS_Z_ACCEL_VALID)) {
            geometry_msgs::msg::Vector3Stamped accel_msg;
            accel_msg.header.stamp    = stamp;
            accel_msg.header.frame_id = "imu";
            accel_msg.vector.x = (ins.bitfield0 & UBX_ESF_INS_X_ACCEL_VALID)
                                  ? ins.xAccel * MG_SCALE : 0.0;
            accel_msg.vector.y = (ins.bitfield0 & UBX_ESF_INS_Y_ACCEL_VALID)
                                  ? ins.yAccel * MG_SCALE : 0.0;
            accel_msg.vector.z = (ins.bitfield0 & UBX_ESF_INS_Z_ACCEL_VALID)
                                  ? ins.zAccel * MG_SCALE : 0.0;
            accel_pub_->publish(accel_msg);
        }
    }

    /**
     * @brief Send CFG-MSG to enable UBX-ESF-INS and disable UBX-ESF-RAW.
     *
     * ESF-INS provides bias-compensated angular rates and accelerations from
     * the INS fusion engine.  ESF-RAW carries uncompensated sensor data and
     * is disabled to reduce bandwidth and avoid confusion.
     */
    void enable_esf_ins_output()
    {
        std::lock_guard<std::mutex> lock(socket_write_mutex_);
        if (socket_fd_ < 0) return;

        // Enable ESF-INS (0x10 0x15) at nav rate on all interfaces
        uint8_t cfg_ins[8] = { 0x10, 0x15, 1, 1, 1, 1, 1, 0 };
        send_ubx_message(0x06, 0x01, cfg_ins, sizeof(cfg_ins));

        // Disable ESF-RAW (0x10 0x03) — noisy, uncompensated, not needed
        uint8_t cfg_raw[8] = { 0x10, 0x03, 0, 0, 0, 0, 0, 0 };
        send_ubx_message(0x06, 0x01, cfg_raw, sizeof(cfg_raw));

        RCLCPP_INFO(this->get_logger(),
            "Sent CFG-MSG: enabled ESF-INS (compensated dynamics), disabled ESF-RAW");
    }

    /**
     * @brief Dedicated 50 Hz sender thread for UBX-ESF-MEAS
     *
     * Sends at a fixed isochronous 20 ms interval so the u-blox receiver can
     * reliably auto-detect the sampling frequency for scale factor estimation.
     * Reads latest_speed_mm_s_ set by the vehicle_state_callback.
     */
    void sender_loop()
    {
        using namespace std::chrono;
        auto next = steady_clock::now();

        while (running_) {
            next += milliseconds(20);  // 50 Hz
            std::this_thread::sleep_until(next);

            if (!running_) break;  // check after waking so we don't send on a closing socket

            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            if (socket_fd_ < 0) {
                continue;
            }

            // Stop sending if vehicle state has not arrived for >1 second
            int64_t last_vs = last_vehicle_state_mono_ns_.load();
            if (last_vs == 0) {
                continue;  // never received
            }
            int64_t age_ms = (std::chrono::steady_clock::now().time_since_epoch().count()
                              - last_vs) / 1000000LL;
            if (age_ms > 1000) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Vehicle state stale (%lld ms) - ESF-MEAS paused",
                    static_cast<long long>(age_ms));
                continue;
            }

            int32_t speed_mm_s = latest_speed_mm_s_.load();
            uint32_t time_tag  = latest_timestamp_ms_.load();

            // Pack per UBX spec: bits[31:24] = dataType(11), bits[23:0] = signed dataField
            uint32_t data_word = (11u << 24) | (static_cast<uint32_t>(speed_mm_s) & 0x00FFFFFF);

            uint16_t flags = static_cast<uint16_t>(1u << 11);  // numMeas=1 in bits[15:11]
            uint16_t sensor_id = 0;

            uint8_t payload[12];
            std::memcpy(payload + 0, &time_tag,   4);
            std::memcpy(payload + 4, &flags,      2);
            std::memcpy(payload + 6, &sensor_id,  2);
            std::memcpy(payload + 8, &data_word,  4);

            send_ubx_message(0x10, 0x02, payload, sizeof(payload));

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "ESF-MEAS: speed=%.3f m/s -> word 0x%08X",
                speed_mm_s * 1e-3f, data_word);
        }
    }

    /**
     * @brief Callback for /vehicle/state - stores latest wheel speed for sender thread
     */
    void vehicle_state_callback(const car_control::msg::VehicleState::SharedPtr msg)
    {
        float speed_mps = (msg->rear_wheel_speed_left + msg->rear_wheel_speed_right) / 2.0f;
        int32_t speed_mm_s = static_cast<int32_t>(speed_mps * 1000.0f);
        // Clamp to 24-bit signed range
        speed_mm_s = std::max(-8388608, std::min(8388607, speed_mm_s));
        latest_speed_mm_s_.store(speed_mm_s);
        // Record receive time (monotonic ns) so sender_loop can detect a stale source
        last_vehicle_state_mono_ns_.store(
            std::chrono::steady_clock::now().time_since_epoch().count());

        // Build a relative ms timestamp from nanoseconds-since-boot.
        // Starts at 0 on first message, increments at comma's ~10ms rate.
        // Consistent deltas let the receiver auto-detect the sampling frequency.
        int64_t ts_ns = msg->timestamp;
        int64_t first = first_timestamp_ns_.load();
        if (first == 0) {
            first_timestamp_ns_.store(ts_ns);
            first = ts_ns;
        }
        uint32_t rel_ms = static_cast<uint32_t>((ts_ns - first) / 1000000LL);
        latest_timestamp_ms_.store(rel_ms);
    }

    /**
     * @brief Build and send a UBX message over the TCP socket
     * @param msg_class  UBX message class byte
     * @param msg_id     UBX message ID byte
     * @param payload    Pointer to payload bytes
     * @param payload_len Number of payload bytes
     * 
     * Caller must hold socket_write_mutex_.
     */
    /** Build and send a UBX frame. Uses ubx_build_message() from ubx_protocol.hpp. */
    void send_ubx_message(uint8_t msg_class, uint8_t msg_id,
                          const uint8_t* payload, uint16_t payload_len)
    {
        auto msg = ubx_build_message(msg_class, msg_id, payload, payload_len);
        ssize_t sent = send(socket_fd_, msg.data(), msg.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "ESF-MEAS send failed: %s", strerror(errno));
        }
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<car_control::msg::EsfStatus>::SharedPtr esf_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
    
    // Reader / sender threads
    std::thread reader_thread_;
    std::thread sender_thread_;
    std::atomic<bool> running_;
    
    // TCP connection
    int socket_fd_;
    std::vector<uint8_t> rx_buffer_;
    
    // Origin for local coordinate conversion
    bool origin_set_;
    double origin_lat_;
    double origin_lon_;
    double origin_alt_;
    
    // Vehicle state subscriber (feeds odometer data to u-blox)
    rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr vehicle_state_sub_;
    std::atomic<int32_t>  latest_speed_mm_s_{0};          // written by callback, read by sender thread
    std::atomic<uint32_t> latest_timestamp_ms_{0};         // relative ms since first comma message
    std::atomic<int64_t>  first_timestamp_ns_{0};          // first comma timestamp (nanoseconds)
    std::atomic<int64_t>  last_vehicle_state_mono_ns_{0};  // steady_clock ns of last /vehicle/state msg
    std::mutex socket_write_mutex_;

    // IMU calibration status (for ADR mode)
    uint8_t fusion_mode_;        // 0=init, 1=fusion, 2=suspended, 3=disabled
    uint8_t alignment_status_;   // From ESF-ALG flags
    bool calibration_complete_;
    bool fusion_disabled_warned_;
    // Heading state
    double last_valid_enu_yaw_  = 0.0;  // ENU yaw [rad], held when headVehValid drops
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSNode>());
    rclcpp::shutdown();
    return 0;
}
