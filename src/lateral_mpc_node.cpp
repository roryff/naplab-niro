/**
 * @file lateral_mpc_node.cpp
 * @brief Unified lateral MPC for ROS2 autonomous car.
 *
 * Replaces path_follower_node + steering_mpc_node with a single node that
 * performs path following and steering torque control in one OSQP optimisation.
 *
 * 4-state model per step k: [CTE_k, dPsi_k, delta_k, dRate_k]
 *   CTE   — cross-track error [m]  (positive = car left of path)
 *   dPsi  — heading error [rad]    (positive = car pointing right of path)
 *   delta — front-axle steer [rad]
 *   dRate — steering rate [rad/s]
 * Input u_k — steering torque [-1, 1]
 *
 * Subscriptions:
 *   gnss/pose             (geometry_msgs/PoseStamped)  — ENU position + yaw
 *   vehicle/state         (car_control/VehicleState)   — v_ego [km/h], steering_angle_deg [sw-deg]
 *   enable_path_following (std_msgs/Bool)              — rising edge starts, any msg stops
 *   gnss/yaw_rate         (std_msgs/Float32)           — IMU yaw rate [rad/s] (optional)
 *
 * Publications:
 *   cmd_vel                       (geometry_msgs/Twist)   — linear.x=accel, angular.z=torque
 *   lateral_mpc/cte_m             (std_msgs/Float64)
 *   lateral_mpc/heading_error_deg (std_msgs/Float64)
 *   lateral_mpc/desired_delta_deg (std_msgs/Float64)      — curvature-based feedforward [deg fw]
 *   lateral_mpc/actual_delta_deg  (std_msgs/Float64)      — measured front-axle steer [deg fw]
 *   lateral_mpc/torque_cmd        (std_msgs/Float64)
 *   lateral_mpc/progress_m        (std_msgs/Float64)
 *   path_following_status         (std_msgs/Bool)         — latched
 *   path_visualization            (nav_msgs/Path)         — latched
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include "car_control/msg/vehicle_state.hpp"
#include <nav_msgs/msg/path.hpp>
#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>
#include <deque>

// ============================================================
// Vehicle & control constants
// ============================================================

static constexpr double CONTROL_HZ          = 20.0;
static constexpr double DT                  = 1.0 / CONTROL_HZ;  // 0.05 s
static constexpr double WHEELBASE           = 2.79;              // Kia Niro [m]
static constexpr double STEERING_RATIO      = 15.33;             // sw-deg per road-wheel deg
static constexpr double MIN_SPEED           = 0.3;               // [m/s] stop threshold
static constexpr double SOFT_START_DURATION = 3.0;               // ramp gains over first N s

// ============================================================
// Piecewise-linear path in ENU frame
// Identical to path_follower_node.cpp – do not modify.
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
     * Savitzky-Golay smoothing (window=9, order=3) applied to x/y coordinates.
     * Reduces GPS position noise in recorded paths without distorting large-scale
     * geometry. Arc-lengths are recomputed from the smoothed coordinates.
     * Edge handling: clamp — boundary points use the nearest valid index.
     */
    void smooth(int passes = 2)
    {
        // SG(9,3) symmetric coefficients: [-21,14,39,54,59,54,39,14,-21] / 231
        constexpr int    W    = 9;
        constexpr int    H    = W / 2;   // 4
        constexpr double c[W] = { -21.0, 14.0, 39.0, 54.0, 59.0,
                                    54.0, 39.0, 14.0, -21.0 };
        constexpr double norm = 231.0;

        const size_t n = wpts_.size();
        if (n < static_cast<size_t>(W)) return;

        std::vector<double> xs(n), ys(n);
        for (int pass = 0; pass < passes; ++pass) {
            for (size_t i = 0; i < n; ++i) {
                double sx = 0.0, sy = 0.0;
                for (int k = -H; k <= H; ++k) {
                    size_t j = static_cast<size_t>(
                        std::clamp(static_cast<int>(i) + k, 0, static_cast<int>(n) - 1));
                    sx += c[k + H] * wpts_[j].first;
                    sy += c[k + H] * wpts_[j].second;
                }
                xs[i] = sx / norm;
                ys[i] = sy / norm;
            }
            for (size_t i = 0; i < n; ++i) wpts_[i] = { xs[i], ys[i] };
        }

        // Recompute arc-lengths from smoothed positions
        s_.resize(n);
        s_[0] = 0.0;
        for (size_t i = 1; i < n; ++i) {
            double dx = wpts_[i].first  - wpts_[i-1].first;
            double dy = wpts_[i].second - wpts_[i-1].second;
            s_[i] = s_[i-1] + std::hypot(dx, dy);
        }
    }

    /** Find arc-length of closest point to (qx, qy). Updates hint in place. */
    double findClosest(double qx, double qy, size_t & hint) const
    {
        if (wpts_.size() < 2) return 0.0;

        double best_sq = std::numeric_limits<double>::max();
        double best_s  = s_[hint];
        size_t end_idx = std::min(wpts_.size() - 1, hint + 300);

        for (size_t i = hint; i < end_idx; ++i) {
            double ax = wpts_[i].first,   ay = wpts_[i].second;
            double bx = wpts_[i+1].first, by = wpts_[i+1].second;
            double dx = bx - ax,          dy = by - ay;
            double seg2 = dx*dx + dy*dy;
            if (seg2 < 1e-12) continue;

            double t = ((qx-ax)*dx + (qy-ay)*dy) / seg2;
            t = std::clamp(t, 0.0, 1.0);

            double px = ax + t*dx, py = ay + t*dy;
            double d2 = (qx-px)*(qx-px) + (qy-py)*(qy-py);

            if (d2 < best_sq) {
                best_sq = d2;
                best_s  = s_[i] + t * std::sqrt(seg2);
                hint    = i;
            }
        }
        return best_s;
    }

    /** Position at arc-length s. */
    std::pair<double,double> position(double s) const { return interp(s); }

    /** Path tangent heading [rad] (ENU: East=0, CCW positive). */
    double heading(double s) const
    {
        constexpr double ds = 0.2;
        auto [x0, y0] = interp(std::max(0.0, s - ds));
        auto [x1, y1] = interp(std::min(totalLength(), s + ds));
        return std::atan2(y1 - y0, x1 - x0);
    }

    /**
     * Signed cross-track error at (qx, qy) for path point at arc-length s.
     * Positive = vehicle is to the LEFT of the path.
     */
    double crossTrackError(double qx, double qy, double s) const
    {
        auto [px, py] = interp(s);
        double h  = heading(s);
        double nx =  std::sin(h);
        double ny = -std::cos(h);
        return (qx - px)*nx + (qy - py)*ny;
    }

    /**
     * Signed heading error: path_heading – car_heading, wrapped to [-π, π].
     * Positive = car pointing right of path.
     */
    double headingError(double s, double car_heading) const
    {
        double err = heading(s) - car_heading;
        while (err >  M_PI) err -= 2.0*M_PI;
        while (err < -M_PI) err += 2.0*M_PI;
        return err;
    }

    /** Path curvature at arc-length s [rad/m]. */
    double curvature(double s) const
    {
        constexpr double ds = 0.5;
        double h0 = heading(std::max(0.0, s - ds));
        double h1 = heading(std::min(totalLength(), s + ds));
        double dh = h1 - h0;
        while (dh >  M_PI) dh -= 2.0 * M_PI;
        while (dh < -M_PI) dh += 2.0 * M_PI;
        return dh / (2.0 * ds);
    }

    const std::vector<std::pair<double,double>>& waypoints() const { return wpts_; }

private:
    std::vector<std::pair<double,double>> wpts_;
    std::vector<double>                   s_;

    std::pair<double,double> interp(double s) const
    {
        if (wpts_.empty()) return {0.0, 0.0};
        if (s <= 0.0)           return wpts_.front();
        if (s >= s_.back())     return wpts_.back();

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
// LateralMpcNode
// ============================================================
class LateralMpcNode : public rclcpp::Node
{
    enum class State { IDLE, FOLLOWING, STOPPING };

public:
    LateralMpcNode()
    : Node("lateral_mpc_node"),
      state_(State::IDLE),
      path_start_time_(this->now()),
      hint_front_(0)
    {
        // ---- Parameters --------------------------------------------------------
        declare_parameter("desired_speed_mps", 4.0);
        declare_parameter("stop_distance",     3.0);
        declare_parameter("horizon",           40);
        declare_parameter("weight_cte",        2.0);   // [1/m²]
        declare_parameter("weight_psi",        2.0);   // [1/rad²]  — was 1.0; increased for better heading tracking
        declare_parameter("weight_torque",     0.1);
        declare_parameter("tau_r",             0.78);  // actuator time constant [s]
        declare_parameter("gain_r",            36.0);  // sw-deg/s per unit torque
        declare_parameter("rate_up",           3.1/3.0);
        declare_parameter("rate_down",         5.5/3.0);
        declare_parameter("torque_limit",      1.0);
        declare_parameter("kp_speed",          0.3);
        declare_parameter("tau_i_cte",         8.0);   // leaky integrator time constant [s]
        declare_parameter("ki_cte",            0.15);  // integrator gain
        declare_parameter<std::string>("path_csv_file", "");
        declare_parameter<bool>("auto_enable", false);

        N_ = static_cast<int>(std::max(1L, std::min(get_parameter("horizon").as_int(), (int64_t)64)));

        // ---- Subscriptions -----------------------------------------------------
        gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "gnss/pose", 10,
            std::bind(&LateralMpcNode::gnssPoseCallback, this, std::placeholders::_1));

        vehicle_state_sub_ = create_subscription<car_control::msg::VehicleState>(
            "vehicle/state", 10,
            std::bind(&LateralMpcNode::vehicleStateCallback, this, std::placeholders::_1));

        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "enable_path_following", 10,
            std::bind(&LateralMpcNode::enableCallback, this, std::placeholders::_1));

        yaw_rate_sub_ = create_subscription<std_msgs::msg::Float32>(
            "gnss/yaw_rate", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                yaw_rate_ = static_cast<double>(msg->data);
            });

        // ---- Publishers --------------------------------------------------------
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        status_pub_   = create_publisher<std_msgs::msg::Bool>(
            "path_following_status", latched_qos);
        path_vis_pub_ = create_publisher<nav_msgs::msg::Path>(
            "path_visualization", latched_qos);

        pub_cte_           = create_publisher<std_msgs::msg::Float64>("lateral_mpc/cte_m",             10);
        pub_hdg_err_       = create_publisher<std_msgs::msg::Float64>("lateral_mpc/heading_error_deg", 10);
        pub_desired_delta_ = create_publisher<std_msgs::msg::Float64>("lateral_mpc/desired_delta_deg", 10);
        pub_actual_delta_  = create_publisher<std_msgs::msg::Float64>("lateral_mpc/actual_delta_deg",  10);
        pub_torque_cmd_    = create_publisher<std_msgs::msg::Float64>("lateral_mpc/torque_cmd",        10);
        pub_progress_      = create_publisher<std_msgs::msg::Float64>("lateral_mpc/progress_m",        10);
        pub_integral_cte_  = create_publisher<std_msgs::msg::Float64>("lateral_mpc/integral_cte",      10);
        pub_lateral_error_ = create_publisher<std_msgs::msg::Float64>("lateral_error",                 10);
        pub_heading_error_ = create_publisher<std_msgs::msg::Float64>("heading_error",                 10);
        pub_pf_cmd_vel_    = create_publisher<geometry_msgs::msg::Twist>("path_follower/cmd_vel",       10);

        // ---- Build path --------------------------------------------------------
        std::string csv_file = get_parameter("path_csv_file").as_string();
        if (!csv_file.empty()) {
            loadPathFromCSV(csv_file);
        } else {
            createSinusoidalPath(500.0, 3.0, 8.0, 80.0, 50.0, 1.0);
        }
        publishPathVisualization();

        // ---- Control timer -----------------------------------------------------
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(DT),
            std::bind(&LateralMpcNode::controlLoop, this));

        auto_enable_ = get_parameter("auto_enable").as_bool();

        RCLCPP_INFO(get_logger(),
            "LateralMpcNode ready.  Path: %.1f m  N=%d  "
            "Publish 'true' on ~/enable_path_following to start.",
            path_.totalLength(), N_);

        if (auto_enable_) {
            RCLCPP_INFO(get_logger(),
                "auto_enable=true: will start automatically after first GNSS fix.");
        }
    }

    ~LateralMpcNode()
    {
        if (osqp_solver_) {
            osqp_cleanup(osqp_solver_);
            osqp_solver_ = nullptr;
        }
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

        const auto & q = msg->pose.orientation;
        car_heading_ = std::atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z));

        bool was_valid = gnss_valid_;
        gnss_valid_ = true;

        if (auto_enable_ && !was_valid && !auto_enable_fired_) {
            auto_enable_fired_ = true;
            auto_enable_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    auto_enable_timer_.reset();  // one-shot
                    auto m = std::make_shared<std_msgs::msg::Bool>();
                    m->data = true;
                    enableCallback(m);
                });
        }
    }

    void vehicleStateCallback(const car_control::msg::VehicleState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        car_speed_mps_ = static_cast<double>(msg->v_ego) / 3.6;

        // Convert steering-wheel angle [sw-deg] → front-axle angle [rad]
        double new_delta = static_cast<double>(msg->steering_angle_deg)
                           * (M_PI / 180.0) / STEERING_RATIO;

        double t = this->now().seconds();
        if (prev_delta_time_ > 0.0) {
            double dt_meas = t - prev_delta_time_;
            if (dt_meas > 0.0 && dt_meas < 1.0) {
                car_delta_rate_ = (new_delta - prev_delta_rad_) / dt_meas;
            }
        }
        prev_delta_rad_  = new_delta;
        prev_delta_time_ = t;
        car_delta_rad_   = new_delta;
    }

    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            // false → stop if currently active (matches cascade behaviour)
            if (state_ == State::FOLLOWING || state_ == State::STOPPING) {
                RCLCPP_INFO(get_logger(), "Path following STOPPED by user.");
                state_ = State::IDLE;
                publishCmd(0.0, 0.0);
                publishStatus(false);
            }
            return;
        }

        // true → toggle: start if IDLE, stop if already active
        if (state_ == State::IDLE) {
            if (!gnss_valid_) {
                RCLCPP_WARN(get_logger(), "Cannot start: no GNSS fix received yet.");
                return;
            }
            if (path_.isEmpty()) {
                RCLCPP_WARN(get_logger(), "Cannot start: path is empty.");
                return;
            }
            hint_front_      = 0;
            u_prev_          = 0.0;
            integral_cte_    = 0.0;
            path_start_time_ = this->now();
            state_           = State::FOLLOWING;
            RCLCPP_INFO(get_logger(), "Path following STARTED.");
            publishStatus(true);

        } else if (state_ == State::FOLLOWING || state_ == State::STOPPING) {
            RCLCPP_INFO(get_logger(), "Path following STOPPED by user.");
            state_ = State::IDLE;
            publishCmd(0.0, 0.0);
            publishStatus(false);
        }
    }

    // =========================================================================
    // Control loop  (20 Hz)
    // =========================================================================

    void controlLoop()
    {
        if (state_ == State::IDLE) return;

        double car_x, car_y, car_heading, car_speed, car_delta, car_delta_rate;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            car_x          = car_x_;
            car_y          = car_y_;
            car_heading    = car_heading_;
            car_speed      = car_speed_mps_;
            car_delta      = car_delta_rad_;
            car_delta_rate = car_delta_rate_;
        }

        // --- Stopping check --------------------------------------------------
        if (state_ == State::STOPPING && car_speed <= MIN_SPEED) {
            RCLCPP_INFO(get_logger(), "Path end reached.  Returning to IDLE.");
            state_ = State::IDLE;
            publishCmd(0.0, 0.0);
            publishStatus(false);
            return;
        }

        // --- Project rear axle (GNSS antenna) onto path ----------------------
        // CTE and heading are referenced at the rear axle (GNSS position) to avoid
        // the geometric inside-corner bias caused by a front-axle reference point.
        double s_rear = path_.findClosest(car_x, car_y, hint_front_);

        // --- Check if approaching end of path --------------------------------
        double remaining = path_.totalLength() - s_rear;
        if (remaining < get_parameter("stop_distance").as_double() &&
            state_ == State::FOLLOWING)
        {
            RCLCPP_INFO(get_logger(),
                "Approaching path end (%.1f m remaining).  Slowing down...", remaining);
            state_ = State::STOPPING;
        }

        // --- Soft-start ramp --------------------------------------------------
        double elapsed = (this->now() - path_start_time_).seconds();
        double ramp    = std::min(1.0, elapsed / SOFT_START_DURATION);

        // --- Initial state for MPC (rear-axle reference) ----------------------
        double cte  = path_.crossTrackError(car_x, car_y, s_rear);
        double dpsi = path_.headingError(s_rear, car_heading);

        // Curvature-based desired steer angle (feedforward reference for debug only)
        double desired_delta_rad = path_.curvature(s_rear) * WHEELBASE;

        // --- Leaky CTE integrator (slow integral action for steady-state offset) ---
        // Accumulates persistent CTE bias over τ_i seconds; corrects road camber /
        // EPS deadband offsets without affecting the fast lateral dynamics.
        {
            const double tau_i = get_parameter("tau_i_cte").as_double();
            integral_cte_ = integral_cte_ * (1.0 - DT / tau_i) + DT * cte;
        }
        double cte_biased = cte + get_parameter("ki_cte").as_double() * integral_cte_;

        // --- Solve MPC -------------------------------------------------------
        double torque_cmd = 0.0;
        if (state_ == State::FOLLOWING) {
            torque_cmd = ramp * solveMpc(cte_biased, dpsi, car_delta, car_delta_rate,
                                         car_speed, s_rear, yaw_rate_);
        }
        torque_cmd = std::clamp(torque_cmd, -1.0, 1.0);
        u_prev_    = torque_cmd;

        // --- Speed P-controller ----------------------------------------------
        double desired_speed = (state_ == State::STOPPING) ? 0.0
            : get_parameter("desired_speed_mps").as_double();
        double kp_speed = get_parameter("kp_speed").as_double();
        double accel_cmd = std::clamp(kp_speed * (desired_speed - car_speed), -1.0, 1.0);

        // --- Publish ---------------------------------------------------------
        publishCmd(accel_cmd, torque_cmd);

        auto f64 = [](double v) { std_msgs::msg::Float64 m; m.data = v; return m; };
        pub_cte_          ->publish(f64(cte));
        pub_hdg_err_      ->publish(f64(dpsi * 180.0 / M_PI));
        pub_desired_delta_->publish(f64(desired_delta_rad * 180.0 / M_PI));
        pub_actual_delta_ ->publish(f64(car_delta * 180.0 / M_PI));
        pub_torque_cmd_   ->publish(f64(torque_cmd));
        pub_progress_     ->publish(f64(s_rear));
        pub_integral_cte_ ->publish(f64(integral_cte_));

        // Dashboard-compatible topics (mirror cascade node interface)
        pub_lateral_error_->publish(f64(cte));
        pub_heading_error_->publish(f64(dpsi));   // [rad] – dashboard calls math.degrees()
        {
            geometry_msgs::msg::Twist pf_cmd;
            pf_cmd.linear.x  = desired_speed;
            pf_cmd.angular.z = desired_delta_rad;  // front-axle [rad]
            pub_pf_cmd_vel_->publish(pf_cmd);
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
            "[%s]  s=%.1f/%.1f m | CTE=%.3f m | dPsi=%.2f° | "
            "delta=%.2f° | torque=%.3f | v=%.2f m/s",
            (state_ == State::FOLLOWING) ? "FOLLOWING" : "STOPPING",
            s_rear, path_.totalLength(),
            cte, dpsi * 180.0 / M_PI,
            car_delta * 180.0 / M_PI,
            torque_cmd, car_speed);
    }

    // =========================================================================
    // Unified lateral MPC
    //
    // Decision variables: U = [u_0, ..., u_{N-1}]  (steering torques)
    //
    // Cost:  Σ_{k=0}^{N-1} [ w_cte * CTE_{k+1}²  +  w_psi * dPsi_{k+1}²
    //                       + w_torque * u_k² ]
    //
    // Dynamics (linearised, front-axle rad units):
    //   dRate_{k+1} = a_rate * dRate_k + b_rate * u_k
    //   delta_{k+1} = delta_k + dt * dRate_{k+1}
    //   dPsi_{k+1}  = dPsi_k  + (v * delta_k / L - kappa_k * v) * dt
    //   CTE_{k+1}   = CTE_k   + v * dPsi_k * dt
    //
    // Propagation: each state = constant + G·U  (linear in U)
    // =========================================================================

    double solveMpc(double cte0, double dpsi0, double delta0, double drate0,
                    double v,    double s_ref,  double r_measured = 0.0)
    {
        const int n = N_;
        if (n < 1) return u_prev_;

        const double dt        = DT;
        const double tau_r     = get_parameter("tau_r").as_double();
        const double gain_r    = get_parameter("gain_r").as_double();
        const double w_cte     = get_parameter("weight_cte").as_double();
        const double w_psi     = get_parameter("weight_psi").as_double();
        const double w_t       = get_parameter("weight_torque").as_double();
        const double rate_up   = get_parameter("rate_up").as_double();
        const double rate_down = get_parameter("rate_down").as_double();
        const double tlim      = get_parameter("torque_limit").as_double();

        // Actuator dynamics coefficients (front-axle rad units)
        const double alpha      = dt / (tau_r + dt);
        const double a_rate     = 1.0 - alpha;
        // gain_r: 36 sw-deg/s/unit → rad/s/unit at front axle
        const double gain_r_rad = gain_r * (M_PI / 180.0) / STEERING_RATIO;
        const double b_rate     = alpha * gain_r_rad;

        const double v_eff = std::max(0.5, v);  // guard against division by zero at low speed

        // ---- Build QP cost matrices (upper-triangular P, gradient q) --------
        const int P_nz = n * (n + 1) / 2;
        std::vector<OSQPFloat> P_x(P_nz, 0.0);
        std::vector<OSQPInt>   P_i(P_nz, 0);
        std::vector<OSQPInt>   P_p(n + 1, 0);
        for (int j = 0; j < n; j++) P_p[j + 1] = P_p[j] + (j + 1);
        std::vector<OSQPFloat> q_vec(n, 0.0);

        // P_i: row indices for upper-triangular column-major storage
        for (int j = 0; j < n; j++)
            for (int i = 0; i <= j; i++)
                P_i[P_p[j] + i] = i;

        // ---- State propagation: x_{k+1} = c_{k+1} + G_{k+1} · U -----------
        double c_rate  = drate0;
        double c_delta = delta0;
        double c_psi   = dpsi0;
        double c_cte   = cte0;

        std::vector<double> G_rate (n, 0.0);
        std::vector<double> G_delta(n, 0.0);
        std::vector<double> G_psi  (n, 0.0);
        std::vector<double> G_cte  (n, 0.0);

        for (int k = 0; k < n; k++) {
            // Path curvature feedforward at predicted vehicle position for step k
            double s_k   = std::min(s_ref + k * dt * v_eff, path_.totalLength());
            double kappa = path_.curvature(s_k);

            // dRate_{k+1} = a_rate * dRate_k + b_rate * u_k
            double c_rate_new = a_rate * c_rate;
            std::vector<double> G_rate_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_rate_new[j] = a_rate * G_rate[j] + (j == k ? b_rate : 0.0);

            // delta_{k+1} = delta_k + dt * dRate_{k+1}
            double c_delta_new = c_delta + dt * c_rate_new;
            std::vector<double> G_delta_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_delta_new[j] = G_delta[j] + dt * G_rate_new[j];

            // dPsi_{k+1} = dPsi_k - (yaw_rate - kappa*v) * dt
            // For k=0: use measured yaw rate (from IMU) when available for accuracy.
            // For k>0: use kinematic model v*delta/L (no measured future yaw rate).
            double yaw_rate_k = (k == 0 && std::abs(r_measured) > 1e-6)
                                ? r_measured
                                : v_eff * c_delta / WHEELBASE;
            double c_psi_new = c_psi - (yaw_rate_k - kappa * v_eff) * dt;
            std::vector<double> G_psi_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_psi_new[j] = G_psi[j] - v_eff * dt / WHEELBASE * G_delta[j];

            // CTE_{k+1} = CTE_k + v * dPsi_k * dt
            // Note: uses dPsi_k (before update)
            double c_cte_new = c_cte + v_eff * c_psi * dt;
            std::vector<double> G_cte_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_cte_new[j] = G_cte[j] + v_eff * dt * G_psi[j];

            // Advance state
            c_rate  = c_rate_new;   G_rate  = G_rate_new;
            c_delta = c_delta_new;  G_delta = G_delta_new;
            c_psi   = c_psi_new;    G_psi   = G_psi_new;
            c_cte   = c_cte_new;    G_cte   = G_cte_new;

            // Accumulate cost from CTE_{k+1} and dPsi_{k+1}
            // ∂²J/∂U_i ∂U_j += 2*w_cte*G_cte[i]*G_cte[j] + 2*w_psi*G_psi[i]*G_psi[j]
            for (int i = 0; i < n; i++) {
                for (int j = i; j < n; j++) {
                    P_x[P_p[j] + i] += static_cast<OSQPFloat>(
                        2.0 * w_cte * G_cte[i] * G_cte[j] +
                        2.0 * w_psi * G_psi[i] * G_psi[j]);
                }
            }
            // ∂J/∂U_j += 2*w_cte*c_cte*G_cte[j] + 2*w_psi*c_psi*G_psi[j]
            for (int j = 0; j < n; j++) {
                q_vec[j] += static_cast<OSQPFloat>(
                    2.0 * w_cte * c_cte * G_cte[j] +
                    2.0 * w_psi * c_psi * G_psi[j]);
            }

            // Torque regularisation for u_k: ∂²J/∂U_k² += 2*w_t
            P_x[P_p[k] + k] += static_cast<OSQPFloat>(2.0 * w_t);
        }

        // ---- Build constraint matrix A (rate-of-change + magnitude) ---------
        // Same structure as steering_mpc_node.cpp:
        //   Rows 4k+0: U_{k-1} - U_k ≤ rate_up*dt   (u_prev for k=0)
        //   Rows 4k+1: U_k - U_{k-1} ≤ rate_down*dt
        //   Rows 4k+2: U_k ≤ tlim
        //   Rows 4k+3: -U_k ≤ tlim
        const int m = 4 * n;
        std::vector<OSQPFloat> A_x;
        std::vector<OSQPInt>   A_i;
        std::vector<OSQPInt>   A_p(n + 1, 0);
        std::vector<OSQPFloat> l_c(m, static_cast<OSQPFloat>(-OSQP_INFTY));
        std::vector<OSQPFloat> u_c(m,  static_cast<OSQPFloat>( OSQP_INFTY));

        u_c[0] = static_cast<OSQPFloat>( u_prev_ + rate_up   * dt);
        u_c[1] = static_cast<OSQPFloat>(-u_prev_ + rate_down * dt);
        u_c[2] = static_cast<OSQPFloat>(tlim);
        u_c[3] = static_cast<OSQPFloat>(tlim);
        for (int k = 1; k < n; k++) {
            u_c[4*k]   = static_cast<OSQPFloat>(rate_up   * dt);
            u_c[4*k+1] = static_cast<OSQPFloat>(rate_down * dt);
            u_c[4*k+2] = static_cast<OSQPFloat>(tlim);
            u_c[4*k+3] = static_cast<OSQPFloat>(tlim);
        }

        int nz = 0;
        for (int j = 0; j < n; j++) {
            A_p[j] = nz;
            // Own-step: row 4j (rate-up), 4j+1 (rate-down), 4j+2 (tlim upper), 4j+3 (tlim lower)
            A_x.push_back( 1.0f); A_i.push_back(4 * j);
            A_x.push_back(-1.0f); A_i.push_back(4 * j + 1);
            A_x.push_back( 1.0f); A_i.push_back(4 * j + 2);
            A_x.push_back(-1.0f); A_i.push_back(4 * j + 3);
            nz += 4;
            if (j < n - 1) {
                // Forward coupling: -U_j in (U_{j+1} - U_j ≤ rate_up*dt)  → row 4(j+1)
                A_x.push_back(-1.0f); A_i.push_back(4 * (j + 1));
                // Forward coupling: +U_j in (U_j - U_{j+1} ≤ rate_down*dt) → row 4(j+1)+1
                A_x.push_back( 1.0f); A_i.push_back(4 * (j + 1) + 1);
                nz += 2;
            }
        }
        A_p[n] = nz;

        // ---- OSQP setup & solve ---------------------------------------------
        OSQPCscMatrix P_csc;
        P_csc.m = n; P_csc.n = n; P_csc.nzmax = P_nz; P_csc.nz = -1;
        P_csc.x = P_x.data(); P_csc.i = P_i.data(); P_csc.p = P_p.data();
        P_csc.owned = 0;

        OSQPCscMatrix A_csc;
        A_csc.m = m; A_csc.n = n;
        A_csc.nzmax = static_cast<OSQPInt>(A_x.size()); A_csc.nz = -1;
        A_csc.x = A_x.data(); A_csc.i = A_i.data(); A_csc.p = A_p.data();
        A_csc.owned = 0;

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = 0;

        if (osqp_solver_) {
            osqp_cleanup(osqp_solver_);
            osqp_solver_ = nullptr;
        }
        OSQPInt err = osqp_setup(&osqp_solver_, &P_csc, q_vec.data(),
                                 &A_csc, l_c.data(), u_c.data(), m, n, &settings);
        if (err != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "OSQP setup failed (err=%d); holding previous torque.", static_cast<int>(err));
            return u_prev_;
        }

        osqp_solve(osqp_solver_);
        OSQPInt status_val = osqp_solver_->info->status_val;
        if (status_val != OSQP_SOLVED && status_val != OSQP_SOLVED_INACCURATE) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "OSQP solve failed (status=%d); holding previous torque.",
                static_cast<int>(status_val));
            return u_prev_;
        }

        double u0 = osqp_solver_->solution->x[0];
        return std::clamp(u0, -tlim, tlim);
    }

    // =========================================================================
    // Path generation  (copied from path_follower_node)
    // =========================================================================

    void createSinusoidalPath(double total_length,
                               double init_amp,  double final_amp,
                               double init_wl,   double final_wl,
                               double spacing)
    {
        path_.clear();
        hint_front_    = 0;
        integral_cte_  = 0.0;

        const double sinusoid_len = total_length - 60.0;
        const int    n_sin        = static_cast<int>(sinusoid_len / spacing);
        const int    n_tot        = static_cast<int>(total_length / spacing);

        path_.addWaypoint(0.0, 0.0);

        for (int i = 1; i <= n_sin; ++i) {
            double x        = i * spacing;
            double progress = x / sinusoid_len;
            double amp      = init_amp + (final_amp - init_amp) * progress;
            double wl       = init_wl  + (final_wl  - init_wl)  * progress;
            double y        = amp * std::sin(2.0 * M_PI / wl * x);
            path_.addWaypoint(x, y);
        }

        const double last_s   = sinusoid_len;
        const double last_amp = init_amp + (final_amp - init_amp) * 1.0;
        const double last_wl  = init_wl  + (final_wl  - init_wl)  * 1.0;
        const double last_y   = last_amp * std::sin(2.0 * M_PI / last_wl * last_s);
        constexpr double TAPER_LEN = 15.0;

        for (int i = n_sin + 1; i <= n_tot; ++i) {
            double x     = i * spacing;
            double d_end = x - sinusoid_len;
            double taper = std::max(0.0, 1.0 - d_end / TAPER_LEN);
            path_.addWaypoint(x, last_y * taper);
        }

        RCLCPP_INFO(get_logger(),
            "Sinusoidal path created: %.1f m total, %d waypoints.",
            path_.totalLength(), n_tot + 1);
    }

    void loadPathFromCSV(const std::string& filename)
    {
        path_.clear();
        hint_front_   = 0;
        integral_cte_ = 0.0;

        std::ifstream f(filename);
        if (!f.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open path CSV: %s", filename.c_str());
            return;
        }

        int count = 0;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            if (line[0] == 'x' || line[0] == 'X') continue;

            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream ss(line);
            double x = 0.0, y = 0.0;
            if (!(ss >> x >> y)) continue;
            path_.addWaypoint(x, y);
            ++count;
        }

        if (count < 2) {
            RCLCPP_ERROR(get_logger(),
                "CSV '%s' has fewer than 2 waypoints – falling back to sinusoidal path.",
                filename.c_str());
            createSinusoidalPath(500.0, 3.0, 8.0, 80.0, 50.0, 1.0);
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Loaded path from '%s': %d waypoints, %.1f m total.",
            filename.c_str(), count, path_.totalLength());

        path_.smooth();
        RCLCPP_INFO(get_logger(), "Path smoothed (SG 9-pt order-3, 2 passes). New length: %.1f m.",
            path_.totalLength());
    }

    // =========================================================================
    // Publish helpers
    // =========================================================================

    void publishCmd(double accel_cmd, double torque_cmd)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = accel_cmd;   // accel [-1, 1]
        cmd.angular.z = torque_cmd;  // steering torque [-1, 1]
        cmd_vel_pub_->publish(cmd);
    }

    void publishStatus(bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        status_pub_->publish(msg);
    }

    void publishPathVisualization()
    {
        if (path_.isEmpty()) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = this->now();
        path_msg.header.frame_id = "map";

        const double total  = path_.totalLength();
        const int    N_vis  = 200;
        const double step   = total / (N_vis - 1);

        for (int i = 0; i < N_vis; ++i) {
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
    State        state_;
    rclcpp::Time path_start_time_;

    // Vehicle state (mutex-protected; written by callbacks, read by control loop)
    std::mutex data_mutex_;
    double car_x_          = 0.0;
    double car_y_          = 0.0;
    double car_heading_    = 0.0;
    double car_speed_mps_  = 0.0;
    double car_delta_rad_  = 0.0;   // front-axle steer angle [rad]
    double car_delta_rate_ = 0.0;   // steer rate [rad/s], estimated from history
    double prev_delta_rad_ = 0.0;
    double prev_delta_time_= 0.0;
    bool   gnss_valid_         = false;
    bool   auto_enable_        = false;
    bool   auto_enable_fired_  = false;

    // MPC state
    double u_prev_        = 0.0;
    double integral_cte_  = 0.0;  // leaky integrator for steady-state CTE bias correction
    double yaw_rate_      = 0.0;  // measured yaw rate from IMU [rad/s], 0 if not available
    int    N_             = 40;

    // Path
    Path   path_;
    size_t hint_front_ = 0;

    // OSQP workspace (owned; cleaned up in destructor and re-created each solve)
    OSQPSolver* osqp_solver_ = nullptr;

    // ROS 2 handles
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  gnss_pose_sub_;
    rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr   vehicle_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           yaw_rate_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_cte_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_hdg_err_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_desired_delta_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_actual_delta_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_torque_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_progress_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_integral_cte_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_lateral_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_heading_error_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_pf_cmd_vel_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr auto_enable_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LateralMpcNode>());
    rclcpp::shutdown();
    return 0;
}
