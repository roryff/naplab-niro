/**
 * @file path_follower_node.cpp
 * @brief ROS 2 path follower node using Stanley lateral control.
 *
 * Inputs
 * ------
 *   gnss/pose        (geometry_msgs/PoseStamped)   – ENU position + yaw from gnss_node
 *   vehicle/state    (car_control/VehicleState)     – speed, steering angle from comma_node
 *   enable_path_following (std_msgs/Bool)           – rising edge starts, any message while
 *                                                     active stops path following
 *
 * Outputs (to future torque-control node)
 * ----------------------------------------
 *   cmd_vel          (geometry_msgs/Twist)
 *       linear.x   = desired forward speed  [m/s]
 *       angular.z  = desired front-axle steering angle  [rad]  (positive = left)
 *
 * Diagnostics
 * -----------
 *   path_visualization  (nav_msgs/Path)        – sampled path in ENU/map frame
 *   lateral_error       (std_msgs/Float64)     – signed cross-track error [m]
 *   heading_error       (std_msgs/Float64)     – heading error [rad]
 *   path_following_status (std_msgs/Bool)      – true while actively following
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "car_control/msg/vehicle_state.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>
#include <cerrno>
#include <cstring>

// ============================================================
// Tuning constants (overridable via ROS 2 parameters)
// ============================================================

static constexpr double CONTROL_HZ          = 20.0;         // control loop rate
static constexpr double GNSS_STALE_SEC      = 1.5;          // stop if no GNSS for this long
static constexpr double WHEELBASE           = 2.79;         // Kia Niro [m]
static constexpr double MAX_STEER_ANGLE     = 0.5236;       // ≈ 30 deg front axle [rad]
static constexpr double MIN_SPEED           = 0.3;          // [m/s] – stop threshold
static constexpr double DEFAULT_SPEED       = 4.0;          // [m/s]
static constexpr double STOP_DISTANCE       = 3.0;          // distance from end to start stopping [m]
static constexpr double SOFT_START_DURATION = 3.0;          // ramp gains over first N seconds

// Default Stanley gains
static constexpr double DEFAULT_K_PSI      = 1.0;   // heading-error gain
static constexpr double DEFAULT_K_CTE      = 0.5;   // cross-track-error gain
static constexpr double DEFAULT_K_SOFT     = 1.0;   // softening term (avoids div-by-zero)
static constexpr double DEFAULT_K_D_STEER  = 0.1;   // steering-rate damping gain

// ============================================================
// Piecewise-linear path in ENU frame
// ============================================================
class Path
{
public:
    Path() = default;

    void clear()
    {
        wpts_.clear();
        s_.clear();
    }

    bool isEmpty() const { return wpts_.empty(); }

    void addWaypoint(double x, double y)
    {
        if (wpts_.empty()) {
            s_.push_back(0.0);
        } else {
            double dx = x - wpts_.back().first;
            double dy = y - wpts_.back().second;
            s_.push_back(s_.back() + std::hypot(dx, dy));
        }
        wpts_.emplace_back(x, y);
    }

    double totalLength() const
    {
        return s_.empty() ? 0.0 : s_.back();
    }

    /**
     * Find arc-length of the point on the path closest to (qx, qy).
     * @param hint  Start index for search (updated in place; prevents backward jumps).
     */
    double findClosest(double qx, double qy, size_t & hint) const
    {
        if (wpts_.size() < 2) return 0.0;

        double best_sq  = std::numeric_limits<double>::max();
        double best_s   = s_[hint];
        size_t end_idx  = std::min(wpts_.size() - 1, hint + 300);

        for (size_t i = hint; i < end_idx; ++i) {
            double ax = wpts_[i].first,     ay = wpts_[i].second;
            double bx = wpts_[i+1].first,   by = wpts_[i+1].second;
            double dx = bx - ax,            dy = by - ay;
            double seg2 = dx*dx + dy*dy;
            if (seg2 < 1e-12) continue;

            double t = ((qx-ax)*dx + (qy-ay)*dy) / seg2;
            t = std::clamp(t, 0.0, 1.0);

            double px = ax + t*dx, py = ay + t*dy;
            double d2 = (qx-px)*(qx-px) + (qy-py)*(qy-py);

            if (d2 < best_sq) {
                best_sq  = d2;
                best_s   = s_[i] + t * std::sqrt(seg2);
                hint     = i;
            }
        }
        return best_s;
    }

    /** Position at arc-length s. */
    std::pair<double,double> position(double s) const { return interp(s); }

    /** Path tangent heading [rad] (ENU convention: East=0, CCW positive). */
    double heading(double s) const
    {
        constexpr double ds = 0.2;
        auto [x0, y0] = interp(std::max(0.0, s - ds));
        auto [x1, y1] = interp(std::min(totalLength(), s + ds));
        return std::atan2(y1 - y0, x1 - x0);
    }

    /**
     * Signed cross-track error at position (qx, qy) for path point at arc-length s.
     * Positive = vehicle is to the LEFT of the path.
     */
    double crossTrackError(double qx, double qy, double s) const
    {
        auto [px, py] = interp(s);
        double h  = heading(s);
        // Left-hand normal of path direction
        double nx =  std::sin(h);
        double ny = -std::cos(h);
        return (qx - px)*nx + (qy - py)*ny;
    }

    /**
     * Signed heading error: path_heading – car_heading, wrapped to [-π, π].
     * Positive = car is pointing to the right of the path.
     */
    double headingError(double s, double car_heading) const
    {
        double err = heading(s) - car_heading;
        // Wrap
        while (err >  M_PI) err -= 2.0*M_PI;
        while (err < -M_PI) err += 2.0*M_PI;
        return err;
    }

    const std::vector<std::pair<double,double>>& waypoints() const { return wpts_; }

private:
    std::vector<std::pair<double,double>> wpts_;
    std::vector<double>                   s_;

    std::pair<double,double> interp(double s) const
    {
        if (wpts_.empty()) return {0.0, 0.0};
        if (s <= 0.0)              return wpts_.front();
        if (s >= s_.back())        return wpts_.back();

        auto it  = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::distance(s_.begin(), it);
        if (i == 0) return wpts_[0];
        i = std::min(i, wpts_.size() - 1);

        double t = (s_[i] - s_[i-1] > 1e-12) ?
                   (s - s_[i-1]) / (s_[i] - s_[i-1]) : 0.0;
        t = std::clamp(t, 0.0, 1.0);

        return {
            wpts_[i-1].first  + t*(wpts_[i].first  - wpts_[i-1].first),
            wpts_[i-1].second + t*(wpts_[i].second - wpts_[i-1].second)
        };
    }
};

// ============================================================
// PathFollowerNode
// ============================================================
class PathFollowerNode : public rclcpp::Node
{
    enum class State { IDLE, FOLLOWING, STOPPING };

public:
        PathFollowerNode()
        : Node("path_follower_node"),
            state_(State::IDLE),
            path_start_time_(this->now()),
            last_gnss_time_(this->now()),
            hint_front_(0)
    {
        timer_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // ---- Parameters --------------------------------------------------------
        this->declare_parameter("desired_speed_mps", DEFAULT_SPEED);
        this->declare_parameter("stop_distance",     STOP_DISTANCE);
        this->declare_parameter("k_psi",             DEFAULT_K_PSI);
        this->declare_parameter("k_cte",             DEFAULT_K_CTE);
        this->declare_parameter("k_soft",            DEFAULT_K_SOFT);
        this->declare_parameter("k_d_steer",         DEFAULT_K_D_STEER);
        this->declare_parameter("steer_ref_traj_n",  40);
        this->declare_parameter<std::string>("path_csv_file", "");  // empty = sinusoidal

        // ---- Subscriptions -----------------------------------------------------
        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gnss/pose", 10,
            std::bind(&PathFollowerNode::gnssPoseCallback, this, std::placeholders::_1));

        vehicle_state_sub_ = this->create_subscription<car_control::msg::VehicleState>(
            "vehicle/state", 10,
            std::bind(&PathFollowerNode::vehicleStateCallback, this, std::placeholders::_1));

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "enable_path_following", 10,
            std::bind(&PathFollowerNode::enableCallback, this, std::placeholders::_1));

        // ---- Publishers --------------------------------------------------------
        cmd_vel_pub_   = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        path_vis_pub_  = this->create_publisher<nav_msgs::msg::Path>(
            "path_visualization", latched_qos);
        lat_err_pub_   = this->create_publisher<std_msgs::msg::Float64>(
            "lateral_error", 10);
        hdg_err_pub_   = this->create_publisher<std_msgs::msg::Float64>(
            "heading_error", 10);
        status_pub_    = this->create_publisher<std_msgs::msg::Bool>(
            "path_following_status", latched_qos);

        // Debug topics for rosbag / evaluation
        progress_pub_    = this->create_publisher<std_msgs::msg::Float64>("path_follower/progress_m",      10);
        remaining_pub_   = this->create_publisher<std_msgs::msg::Float64>("path_follower/remaining_m",     10);
        steer_cmd_pub_   = this->create_publisher<std_msgs::msg::Float64>("path_follower/steer_cmd_deg",   10);
        car_speed_pub_   = this->create_publisher<std_msgs::msg::Float64>("path_follower/car_speed_mps",   10);
        ramp_pub_        = this->create_publisher<std_msgs::msg::Float64>("path_follower/soft_start_ramp", 10);
        steer_ref_traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "path_follower/steer_ref_traj_deg", 10);

        // ---- Build path --------------------------------------------------------
        //  If path_csv_file is set, load recorded drive.  Otherwise use sinusoidal test path.
        std::string csv_file = this->get_parameter("path_csv_file").as_string();
        if (!csv_file.empty()) {
            loadPathFromCSV(csv_file);
        } else {
            createSinusoidalPath(
                500.0,   // total length [m]
                3.0,     // initial lateral amplitude [m]
                8.0,     // final   lateral amplitude [m]
                80.0,    // initial wavelength [m]
                50.0,    // final   wavelength [m]
                1.0);    // waypoint spacing   [m]
        }

        publishPathVisualization();

        // ---- Control timer -----------------------------------------------------
        using namespace std::chrono_literals;
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / CONTROL_HZ),
            std::bind(&PathFollowerNode::controlLoop, this),
            timer_cb_group_);

        RCLCPP_INFO(this->get_logger(),
            "PathFollowerNode ready.  Path length: %.1f m.  "
            "Publish 'true' on ~/enable_path_following to start.",
            path_.totalLength());
    }

private:
    // =========================================================================
    // Callbacks
    // =========================================================================

    void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        car_x_ = msg->pose.position.x;
        car_y_ = msg->pose.position.y;

        // Extract yaw from quaternion (standard ROS convention)
        const auto & q = msg->pose.orientation;
        car_heading_ = std::atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z));

        gnss_valid_     = true;
        last_gnss_time_ = this->now();
    }

    void vehicleStateCallback(const car_control::msg::VehicleState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // v_ego is in km/h (comma_node converts m/s → km/h)
        car_speed_mps_ = static_cast<double>(msg->v_ego) / 3.6;

        // steering_angle_deg is the steering-wheel angle [deg].
        // Convert to front-axle angle [rad] via the mechanical ratio.
        // Kia Niro: ~460 deg lock-to-lock steering wheel → ~30 deg front axle
        constexpr double RATIO = MAX_STEER_ANGLE / (460.0 * M_PI / 180.0);
        car_steer_rad_ = static_cast<double>(msg->steering_angle_deg) * (M_PI / 180.0) * RATIO;
    }

    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // false = explicit stop request
        if (!msg->data) {
            if (state_ == State::FOLLOWING || state_ == State::STOPPING) {
                RCLCPP_INFO(this->get_logger(), "Path following STOPPED by user.");
                state_ = State::IDLE;
                publishCmd(0.0, 0.0);
                publishStatus(false);
            }
            return;
        }

        if (state_ == State::IDLE) {
            if (!gnss_valid_) {
                RCLCPP_WARN(this->get_logger(), "Cannot start: no GNSS fix received yet.");
                return;
            }
            if (path_.isEmpty()) {
                RCLCPP_WARN(this->get_logger(), "Cannot start: path is empty.");
                return;
            }
            // Reset state for a fresh start
            hint_front_   = 0;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                prev_steer_rad_ = car_steer_rad_;
            }
            path_start_time_ = this->now();
            state_ = State::FOLLOWING;

            RCLCPP_INFO(this->get_logger(), "Path following STARTED.");
            publishStatus(true);

        } else {
            RCLCPP_WARN(this->get_logger(), "Path following already active; send false to stop.");
        }
    }

    // =========================================================================
    // Control loop (20 Hz)
    // =========================================================================
    void controlLoop()
    {
        if (state_ == State::IDLE) return;

        // --- GNSS staleness guard --------------------------------------------
        if ((this->now() - last_gnss_time_).seconds() > GNSS_STALE_SEC) {
            RCLCPP_WARN(this->get_logger(),
                "GNSS data stale (>%.1f s). Stopping path following.", GNSS_STALE_SEC);
            state_ = State::IDLE;
            publishCmd(0.0, 0.0);
            publishStatus(false);
            return;
        }

        // Snapshot latest vehicle state
        double car_x, car_y, car_heading, car_speed, car_steer;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            car_x       = car_x_;
            car_y       = car_y_;
            car_heading = car_heading_;
            car_speed   = car_speed_mps_;
            car_steer   = car_steer_rad_;
        }

        // --- Stopping check --------------------------------------------------
        if (state_ == State::STOPPING && car_speed <= MIN_SPEED) {
            RCLCPP_INFO(this->get_logger(), "Path end reached. Returning to IDLE.");
            state_ = State::IDLE;
            publishCmd(0.0, 0.0);
            publishStatus(false);
            return;
        }

        // --- Project front axle onto path ------------------------------------
        double cos_h  = std::cos(car_heading);
        double sin_h  = std::sin(car_heading);
        double front_x = car_x + WHEELBASE * cos_h;
        double front_y = car_y + WHEELBASE * sin_h;

        double s_front = path_.findClosest(front_x, front_y, hint_front_);

        // Also find rear axle arc-length for heading error reference
        size_t hint_rear = (hint_front_ > 0) ? hint_front_ - 1 : 0;
        double s_rear    = path_.findClosest(car_x, car_y, hint_rear);

        // --- Check if approaching end of path --------------------------------
        double remaining = path_.totalLength() - s_rear;
        if (remaining < this->get_parameter("stop_distance").as_double() &&
            state_ == State::FOLLOWING)
        {
            RCLCPP_INFO(this->get_logger(),
                "Approaching path end (%.1f m remaining). Slowing down...", remaining);
            state_ = State::STOPPING;
        }

        // --- Stanley controller ----------------------------------------------
        // Cross-track error: referenced at front axle
        double e   = path_.crossTrackError(front_x, front_y, s_front);
        // Heading error: referenced at rear axle (more stable)
        double psi = path_.headingError(s_rear, car_heading);

        // Soft-start ramp
        double elapsed = (this->now() - path_start_time_).seconds();
        double ramp    = std::min(1.0, elapsed / SOFT_START_DURATION);

        double k_psi     = ramp * this->get_parameter("k_psi").as_double();
        double k_cte     = ramp * this->get_parameter("k_cte").as_double();
        double k_soft    =        this->get_parameter("k_soft").as_double();
        double k_d_steer = ramp * this->get_parameter("k_d_steer").as_double();

        double steer_cmd   = 0.0;
        if (state_ == State::FOLLOWING) {
            double v_eff  = std::max(0.5, car_speed);  // avoid div-by-zero
            steer_cmd = k_psi * psi
                      + std::atan2(k_cte * e, k_soft + v_eff)
                      + k_d_steer * (prev_steer_rad_ - car_steer);

            steer_cmd = std::clamp(steer_cmd, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        }
        prev_steer_rad_ = car_steer;

        // Publish reference trajectory for MPC (N future steering angles)
        // ref_traj[k] = current desired angle + path heading change from s_front to s_front+k*dt*v
        // Converts road-wheel radians to steering-wheel degrees (ratio 15.33 * 180/pi = 878.8)
        static constexpr double SW_RAD_TO_DEG = 15.33 * (180.0 / M_PI);  // sw-deg per road-wheel rad
        {
            int traj_n = this->get_parameter("steer_ref_traj_n").as_int();
            double traj_dt = 1.0 / CONTROL_HZ;
            double v_traj = std::max(0.5, car_speed);
            double base_heading = path_.heading(s_front);
            double base_sw_deg = steer_cmd * SW_RAD_TO_DEG;  // current desired angle in SW deg

            std_msgs::msg::Float64MultiArray traj_msg;
            traj_msg.data.resize(traj_n);
            for (int k = 0; k < traj_n; ++k) {
                double s_k = std::min(s_front + k * traj_dt * v_traj, path_.totalLength());
                double dh = path_.heading(s_k) - base_heading;
                // Wrap dh to [-pi, pi]
                while (dh >  M_PI) dh -= 2.0 * M_PI;
                while (dh < -M_PI) dh += 2.0 * M_PI;
                traj_msg.data[k] = base_sw_deg + dh * SW_RAD_TO_DEG;
            }
            // Clamp to physical steering limit in SW degrees
            static constexpr double MAX_SW_DEG = MAX_STEER_ANGLE * SW_RAD_TO_DEG;
            for (auto& v : traj_msg.data)
                v = std::clamp(v, -MAX_SW_DEG, MAX_SW_DEG);
            steer_ref_traj_pub_->publish(traj_msg);
        }

        // --- Desired speed ---------------------------------------------------
        double desired_speed = (state_ == State::STOPPING) ? 0.0
            : this->get_parameter("desired_speed_mps").as_double();

        // --- Publish ---------------------------------------------------------
        publishCmd(desired_speed, steer_cmd);
        publishDiagnostics(e, psi);

        // Debug topics
        auto f64 = [](double v) { std_msgs::msg::Float64 m; m.data = v; return m; };
        progress_pub_   ->publish(f64(s_rear));                         // arc-length progress [m]
        remaining_pub_  ->publish(f64(remaining));                      // meters to path end
        steer_cmd_pub_  ->publish(f64(steer_cmd * 180.0 / M_PI));      // front-axle steer cmd [deg]
        car_speed_pub_  ->publish(f64(car_speed));                      // actual vehicle speed [m/s]
        ramp_pub_       ->publish(f64(ramp));                           // soft-start ramp [0..1]

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[%s]  s=%.1f/%.1f m | CTE=%.3f m | Psi=%.2f° | steer_cmd=%.2f° | v=%.2f m/s",
            (state_ == State::FOLLOWING) ? "FOLLOWING" : "STOPPING",
            s_rear, path_.totalLength(),
            e, psi * 180.0 / M_PI,
            steer_cmd * 180.0 / M_PI,
            car_speed);
    }

    // =========================================================================
    // Path generation
    // =========================================================================

    /**
     * @brief Generate a sinusoidal path in ENU starting at the origin (0, 0).
     *
     * The path starts at the ENU origin, which corresponds to the first GNSS fix
     * recorded by gnss_node.  Position the vehicle at that fix before enabling
     * path following.
     *
     * @param total_length   Total arc length of the path      [m]
     * @param init_amp       Lateral amplitude at the start    [m]
     * @param final_amp      Lateral amplitude at the end      [m]
     * @param init_wl        Wavelength at the start           [m]
     * @param final_wl       Wavelength at the end             [m]
     * @param spacing        Distance between waypoints        [m]
     */
    void createSinusoidalPath(double total_length,
                              double init_amp,  double final_amp,
                              double init_wl,   double final_wl,
                              double spacing)
    {
        path_.clear();
        hint_front_ = 0;

        const double sinusoid_len = total_length - 60.0;   // leave a straight run-out
        const int    n_sin        = static_cast<int>(sinusoid_len / spacing);
        const int    n_tot        = static_cast<int>(total_length / spacing);

        // Origin waypoint
        path_.addWaypoint(0.0, 0.0);

        // Sinusoidal section
        for (int i = 1; i <= n_sin; ++i) {
            double x        = i * spacing;
            double progress = x / sinusoid_len;
            double amp      = init_amp + (final_amp - init_amp) * progress;
            double wl       = init_wl  + (final_wl  - init_wl)  * progress;
            double y        = amp * std::sin(2.0 * M_PI / wl * x);
            path_.addWaypoint(x, y);
        }

        // Taper the lateral offset back to zero over 15 m, then run straight
        const double last_s   = sinusoid_len;
        const double last_amp = init_amp + (final_amp - init_amp) * 1.0;
        const double last_wl  = init_wl  + (final_wl  - init_wl)  * 1.0;
        const double last_y   = last_amp * std::sin(2.0 * M_PI / last_wl * last_s);
        constexpr double TAPER_LEN = 15.0;

        for (int i = n_sin + 1; i <= n_tot; ++i) {
            double x      = i * spacing;
            double d_end  = x - sinusoid_len;
            double taper  = std::max(0.0, 1.0 - d_end / TAPER_LEN);
            path_.addWaypoint(x, last_y * taper);
        }

        RCLCPP_INFO(this->get_logger(),
            "Sinusoidal path created: %.1f m total, %d waypoints.",
            path_.totalLength(), n_tot + 1);
    }

    /**
     * @brief Load a path from a CSV file recorded by drive_recorder.
     *
     * Expected CSV format (first line may be a header starting with '#' or 'x'):
     *   x,y
     *   1.23,4.56
     *   ...
     */
    void loadPathFromCSV(const std::string& filename)
    {
        path_.clear();
        hint_front_ = 0;

        std::ifstream f(filename);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open path CSV: %s", filename.c_str());
            return;
        }

        int count = 0;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            // Skip header line that starts with letters
            if (line[0] == 'x' || line[0] == 'X') continue;

            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line);
            double x = 0.0, y = 0.0;
            if (!(ss >> x >> y)) continue;
            path_.addWaypoint(x, y);
            ++count;
        }

        if (count < 2) {
            RCLCPP_ERROR(this->get_logger(),
                "CSV file '%s' has fewer than 2 waypoints – falling back to sinusoidal path.",
                filename.c_str());
            createSinusoidalPath(500.0, 3.0, 8.0, 80.0, 50.0, 1.0);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Loaded path from '%s': %d waypoints, %.1f m total.",
            filename.c_str(), count, path_.totalLength());
    }

    // =========================================================================
    // Publish helpers
    // =========================================================================

    /**
     * Publish the control command.
     * @param desired_speed_mps  Desired forward speed [m/s]
     * @param steer_angle_rad    Desired front-axle steering angle [rad]
     *                           (positive = left, i.e. counter-clockwise yaw rate)
     */
    void publishCmd(double desired_speed_mps, double steer_angle_rad)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = desired_speed_mps;
        cmd.angular.z = steer_angle_rad;
        cmd_vel_pub_->publish(cmd);
    }

    void publishStatus(bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        status_pub_->publish(msg);
    }

    void publishDiagnostics(double lateral_error, double heading_error)
    {
        std_msgs::msg::Float64 lat_msg, hdg_msg;
        lat_msg.data = lateral_error;
        hdg_msg.data = heading_error;
        lat_err_pub_->publish(lat_msg);
        hdg_err_pub_->publish(hdg_msg);
    }

    void publishPathVisualization()
    {
        if (path_.isEmpty()) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = this->now();
        path_msg.header.frame_id = "map";

        const double total = path_.totalLength();
        const int    N     = 200;
        const double step  = total / (N - 1);

        for (int i = 0; i < N; ++i) {
            double s = i * step;
            auto [x, y] = path_.position(s);

            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            ps.pose.position.z = 0.0;

            double h = path_.heading(s);
            ps.pose.orientation.w = std::cos(h / 2.0);
            ps.pose.orientation.x = 0.0;
            ps.pose.orientation.y = 0.0;
            ps.pose.orientation.z = std::sin(h / 2.0);

            path_msg.poses.push_back(ps);
        }

        path_vis_pub_->publish(path_msg);
    }

    // =========================================================================
    // Member variables
    // =========================================================================

    // State machine
    State            state_;
    rclcpp::Time     path_start_time_;

    // Vehicle state (written by callbacks, read by control loop)
    std::mutex       data_mutex_;
    double           car_x_         = 0.0;
    double           car_y_         = 0.0;
    double           car_heading_   = 0.0;
    double           car_speed_mps_ = 0.0;
    double           car_steer_rad_ = 0.0;
    double           prev_steer_rad_= 0.0;
    bool             gnss_valid_        = false;
    rclcpp::Time     last_gnss_time_;

    // Path
    Path             path_;
    size_t           hint_front_;   // search hint – prevents backward jumps

    // ROS 2 handles
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  gnss_pose_sub_;
    rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr   vehicle_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              enable_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              lat_err_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              hdg_err_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 status_pub_;
    // Debug publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              progress_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              remaining_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              steer_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              car_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              ramp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    steer_ref_traj_pub_;

    rclcpp::TimerBase::SharedPtr                                      control_timer_;
    rclcpp::CallbackGroup::SharedPtr                                  timer_cb_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("path_follower_node"),
            "mlockall failed: %s", strerror(errno));
    }

    struct sched_param sp{};
    sp.sched_priority = 65;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("path_follower_node"),
            "SCHED_FIFO failed (not root / no CAP_SYS_NICE).");
    }

    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
