/**
 * @file path_follower_node.cpp
 * @brief Path-following controller for the ROS2 autonomous car.
 *
 * Follows a recorded path with a single node that does both lateral steering
 * (a 4-state torque MPC solved with OSQP) and longitudinal speed control (a
 * curvature-aware speed profile + P-controller) in one 20 Hz control loop.
 * Path geometry, B-spline smoothing and the speed-profile builder live in the
 * shared `Path` module (car_control/path.hpp); this file is the control loop.
 *
 * 3-state model per step k: [CTE_k, dPsi_k, delta_k]
 *   CTE   — cross-track error [m]  (positive = car right of path)
 *   dPsi  — heading error [rad]    (positive = car pointing right of path)
 *   delta — front-axle steer [rad]  (speed-scheduled 1st-order actuator)
 * Input u_k — steering torque [-1, 1]
 *
 * Subscriptions:
 *   gnss/pose             (geometry_msgs/PoseStamped)  — ENU position + yaw
 *   vehicle/state         (car_control/VehicleState)   — v_ego [km/h], steering_angle_deg [sw-deg]
 *   enable_path_following (std_msgs/Bool)              — rising edge starts, any msg stops
 *
 * Publications:
 *   cmd_vel                       (car_control/DriveCommand) — accel [-1,1], torque [-1,1]
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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include "car_control/msg/drive_command.hpp"
#include "car_control/msg/vehicle_state.hpp"
#include "car_control/msg/mpc_status.hpp"
#include <nav_msgs/msg/path.hpp>
#include <osqp.h>
#include <sys/mman.h>

#include "car_control/geo_utils.hpp"
#include "car_control/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <deque>
#include <cerrno>
#include <cstring>

// ============================================================
// sched_fo2 helpers (piecewise-linear interpolation)
// ============================================================

// Piecewise-linear interpolation with clamping at endpoints.
static double schedInterp(const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          double x, double floor_val = 0.0)
{
    if (xs.empty()) return floor_val;
    if (x <= xs.front()) return std::max(floor_val, ys.front());
    if (x >= xs.back())  return std::max(floor_val, ys.back());
    for (size_t i = 1; i < xs.size(); ++i) {
        if (x < xs[i]) {
            double t = (x - xs[i-1]) / (xs[i] - xs[i-1]);
            return std::max(floor_val, ys[i-1] + t*(ys[i] - ys[i-1]));
        }
    }
    return std::max(floor_val, ys.back());
}

// ============================================================
// Vehicle & control constants
// ============================================================

// WHEELBASE and STEERING_RATIO are defined in car_control/path.hpp (shared with Path).
static constexpr double CONTROL_HZ          = 20.0;
static constexpr double DT                  = 1.0 / CONTROL_HZ;  // 0.05 s
static constexpr double MIN_SPEED           = 0.3;               // [m/s] stop threshold
static constexpr double SOFT_START_DURATION = 3.0;               // ramp gains over first N s

// ---- Fixed structural constants (set-once; recompile to change) -------------
static constexpr double PATH_SPLINE_KNOT_M     = 5.0;    // B-spline knot spacing [m]; 0 = disabled
static constexpr double STOP_DISTANCE_M        = 3.0;    // start slowing this far from path end [m]
static constexpr double RATE_RISING            = 1.033;  // |u| increasing (0→±1) [norm/s]
static constexpr double RATE_SINKING           = 1.833;  // |u| decreasing (±1→0) [norm/s]
static constexpr double TORQUE_LIMIT           = 1.0;    // |torque| bound
static constexpr double WEIGHT_COMPLEMENTARITY = 2.0;    // L1 penalty discouraging u_plus & u_minus both nonzero
static constexpr double TERMINAL_WEIGHT_FACTOR = 1.0;    // extra CTE/psi weight at final horizon step (P3)

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
      path_start_time_(this->now())
    {
        timer_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // ---- Parameters --------------------------------------------------------
        declare_parameter("desired_speed_mps", 4.0);
        declare_parameter("horizon",           40);
        declare_parameter("weight_cte",        2.0);   // [1/m²]
        declare_parameter("weight_psi",        1.0);   // [1/rad²]
        declare_parameter("weight_torque",     0.1);
        declare_parameter("kp_speed",          0.3);
        declare_parameter<std::string>("path_csv_file", "");
        declare_parameter<bool>("auto_enable", false);
        declare_parameter("origin_lat", 0.0);
        declare_parameter("origin_lon", 0.0);
        // Speed-scheduled first-order (SchedFO2) actuator model breakpoints.
        declare_parameter<std::vector<double>>("sched_v_kmh", std::vector<double>{});
        declare_parameter<std::vector<double>>("sched_tau_r", std::vector<double>{});
        declare_parameter<std::vector<double>>("sched_kss",   std::vector<double>{});
        declare_parameter("speed_profile_decel_mps2", 1.5);
        declare_parameter("speed_profile_accel_mps2", 0.5);
        declare_parameter("curvature_speed_margin",   0.85);
        declare_parameter("curvature_rate_slew_budget_deg_s", 0.0);  // 0 = disabled
        declare_parameter("v_ref_min_mps",            0.5);
        declare_parameter("speed_lookahead_s",        1.5);
        // Offset-free MPC: lateral-disturbance estimator (P1)
        declare_parameter("cte_integral_gain",   0.6);   // [1/s] leaky-integral gain on CTE
        declare_parameter("cte_integral_limit",  0.5);   // [m/s] clamp on estimated drift (anti-windup)
        // Curvature gate (P1b): |kappa| [rad/m] at which the disturbance estimate is fully
        // faded out. w_hat models a straight-line drift (camber); in corners it injected a
        // phantom drift, so estimate + apply it only on straights and fade it in corners.
        declare_parameter("disturbance_curvature_gate", 0.02);  // ~R<50 m corners gate it off
        // Understeer-corrected feedforward (P2): L_eff(v) = L + Kus*v^2
        declare_parameter("understeer_gradient", 0.003); // [s^2/m]

        N_ = static_cast<int>(std::max(1L, std::min(get_parameter("horizon").as_int(), (int64_t)64)));

        // ---- Actuator model (speed-scheduled first-order, SchedFO2) ------------
        sched_v_kmh_ = get_parameter("sched_v_kmh").as_double_array();
        sched_tau_r_ = get_parameter("sched_tau_r").as_double_array();
        sched_kss_   = get_parameter("sched_kss").as_double_array();
        RCLCPP_INFO(get_logger(),
            "SchedFO2 actuator model: %zu breakpoints, tau_r %.2f..%.2f s",
            sched_v_kmh_.size(),
            sched_tau_r_.empty() ? 0.0 : sched_tau_r_.front(),
            sched_tau_r_.empty() ? 0.0 : sched_tau_r_.back());

        // ---- Subscriptions -----------------------------------------------------
        gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "gnss/pose", 10,
            std::bind(&PathFollowerNode::gnssPoseCallback, this, std::placeholders::_1));

        vehicle_state_sub_ = create_subscription<car_control::msg::VehicleState>(
            "vehicle/state", rclcpp::SensorDataQoS(),
            std::bind(&PathFollowerNode::vehicleStateCallback, this, std::placeholders::_1));

        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "enable_path_following", 10,
            std::bind(&PathFollowerNode::enableCallback, this, std::placeholders::_1));

        // ---- Publishers --------------------------------------------------------
        cmd_vel_pub_ = create_publisher<car_control::msg::DriveCommand>("cmd_vel", 10);

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
        pub_vref_          = create_publisher<std_msgs::msg::Float64>("lateral_mpc/vref_mps",          10);
        pub_lateral_error_ = create_publisher<std_msgs::msg::Float64>("lateral_error",                 10);
        pub_heading_error_ = create_publisher<std_msgs::msg::Float64>("heading_error",                 10);
        pub_pf_cmd_vel_    = create_publisher<geometry_msgs::msg::Twist>("path_follower/cmd_vel",       10);
        pub_status_full_   = create_publisher<car_control::msg::MpcStatus>("lateral_mpc/status",         10);

        // ---- Load-path subscription (runtime path switching) -----------------
        load_path_sub_ = create_subscription<std_msgs::msg::String>(
            "/lateral_mpc/load_path", rclcpp::QoS(1),
            std::bind(&PathFollowerNode::loadPathCallback, this, std::placeholders::_1));

        // ---- Map frame origin (must match gnss_node origin_lat/lon) ---------------
        {
            double lat = get_parameter("origin_lat").as_double();
            double lon = get_parameter("origin_lon").as_double();
            if (lat != 0.0 || lon != 0.0) {
                geo::latlon_to_utm32(lat, lon, map_origin_x_, map_origin_y_);
                RCLCPP_INFO(get_logger(),
                    "Map origin: lat=%.7f lon=%.7f -> UTM32 E=%.2f N=%.2f",
                    lat, lon, map_origin_x_, map_origin_y_);
            }
        }

        // ---- Build path --------------------------------------------------------
        std::string csv_file = get_parameter("path_csv_file").as_string();
        if (!csv_file.empty()) {
            loadPathFromCSV(csv_file);
            publishPathVisualization();
        } else {
            RCLCPP_INFO(get_logger(),
                "No path loaded. Publish a CSV filename to ~/load_path to load one.");
        }

        // ---- Control timer -----------------------------------------------------
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(DT),
            std::bind(&PathFollowerNode::controlLoop, this),
            timer_cb_group_);

        auto_enable_ = get_parameter("auto_enable").as_bool();

        RCLCPP_INFO(get_logger(),
            "PathFollowerNode ready. Path: %.1f m  N=%d  spline_knot=%.2f m. "
            "Publish 'true' on ~/enable_path_following to start.",
            path_.totalLength(), N_, PATH_SPLINE_KNOT_M);

        if (auto_enable_) {
            RCLCPP_INFO(get_logger(),
                "auto_enable=true: will start automatically after first GNSS fix.");
        }
    }

    ~PathFollowerNode()
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
            hint_rear_       = 0;
            u_prev_          = 0.0;
            u_prev_plus_     = 0.0;
            u_prev_minus_    = 0.0;
            w_hat_           = 0.0;
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

        // --- Project rear axle onto path -------------------------------------
        const double ref_x = car_x;
        const double ref_y = car_y;
        const double s_rear = path_.findClosest(ref_x, ref_y, hint_rear_);
        const double s_ref  = s_rear;

        // --- Check if approaching end of path --------------------------------
        double remaining = path_.totalLength() - s_rear;
        if (remaining < STOP_DISTANCE_M && state_ == State::FOLLOWING)
        {
            RCLCPP_INFO(get_logger(),
                "Approaching path end (%.1f m remaining).  Slowing down...", remaining);
            state_ = State::STOPPING;
        }

        // --- Soft-start ramp --------------------------------------------------
        double elapsed = (this->now() - path_start_time_).seconds();
        double ramp    = std::min(1.0, elapsed / SOFT_START_DURATION);

        // --- Initial state for MPC -------------------------------------------
        double cte  = path_.crossTrackError(ref_x, ref_y, s_ref);
        double dpsi = path_.headingError(s_ref, car_heading);

        // Offset-free MPC (P1): leaky-integral estimate of the unmodeled lateral
        // disturbance (camber, steering zero-trim, model bias) as a CTE drift rate
        // [m/s]. Clamped for anti-windup. Fed into the CTE prediction in solveMpc so
        // the controller rejects constant disturbances instead of leaving steady CTE.
        // Anti-windup: only accumulate when the actuator is NOT saturated — while
        // |torque|≈1 the controller can't act on extra error, so integrating it just
        // winds up and causes the slow large-amplitude swings seen on the hard path.
        // Curvature gate (P1b): fade the disturbance estimate to 0 in corners, where a
        // constant-drift model is wrong and was making the MPC predict the corner offset
        // would self-cancel (verified 3x optimism). 1 on straights, 0 for |kappa|>=gate.
        const double kgate = get_parameter("disturbance_curvature_gate").as_double();
        const double curv_gate_now = (kgate > 1e-9)
            ? std::clamp(1.0 - std::abs(path_.curvature(s_ref)) / kgate, 0.0, 1.0) : 1.0;
        mpc_dbg_.integrator_frozen = (std::abs(u_prev_) >= 0.97);
        if (state_ == State::FOLLOWING && !mpc_dbg_.integrator_frozen) {
            const double ki   = get_parameter("cte_integral_gain").as_double();
            const double wmax = get_parameter("cte_integral_limit").as_double();
            // scale integration by the gate so w_hat is learned from straights only
            w_hat_ = std::clamp(w_hat_ + ki * cte * curv_gate_now * DT, -wmax, wmax);
        }

        // Curvature-based desired steer angle (feedforward reference for debug only),
        // understeer-corrected: delta = kappa * L_eff(v), L_eff = L + Kus*v^2 (P2).
        const double kus       = get_parameter("understeer_gradient").as_double();
        const double l_eff_ref = WHEELBASE + kus * car_speed * car_speed;
        double desired_delta_rad = path_.curvature(s_ref) * l_eff_ref;

        // --- Solve MPC -------------------------------------------------------
        double torque_cmd = 0.0;
        if (state_ == State::FOLLOWING) {
            double out_plus = 0.0, out_minus = 0.0;
            torque_cmd = ramp * solveMpc(cte, dpsi, car_delta,
                                         car_speed, s_ref, out_plus, out_minus);
            u_prev_plus_  = ramp * out_plus;
            u_prev_minus_ = ramp * out_minus;
        }
        torque_cmd = std::clamp(torque_cmd, -1.0, 1.0);
        u_prev_    = torque_cmd;

        // --- Speed P-controller ----------------------------------------------
        double desired_speed;
        if (state_ == State::STOPPING) {
            desired_speed = 0.0;
        } else if (path_.hasSpeedProfile()) {
            // Scan vref over [s_ref, s_ref + car_speed * lookahead_s] and take the
            // minimum.  Time-based lookahead so the preview window scales with speed,
            // giving a constant braking-time budget regardless of current velocity.
            double lookahead_m = car_speed * get_parameter("speed_lookahead_s").as_double();
            double s_end = std::min(s_ref + lookahead_m, path_.totalLength());
            double v_lookahead = path_.vref(s_ref);
            for (double ss = s_ref + 1.0; ss <= s_end; ss += 1.0)
                v_lookahead = std::min(v_lookahead, path_.vref(ss));

            desired_speed = std::max(v_lookahead, get_parameter("v_ref_min_mps").as_double());
        } else {
            desired_speed = get_parameter("desired_speed_mps").as_double();
        }
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
        pub_vref_         ->publish(f64(desired_speed));

        // Dashboard-compatible topics (mirror cascade node interface)
        pub_lateral_error_->publish(f64(cte));
        pub_heading_error_->publish(f64(dpsi));   // [rad] – dashboard calls math.degrees()
        {
            geometry_msgs::msg::Twist pf_cmd;
            pf_cmd.linear.x  = desired_speed;
            pf_cmd.angular.z = desired_delta_rad;  // front-axle [rad]
            pub_pf_cmd_vel_->publish(pf_cmd);
        }

        publishMpcStatus(s_rear, s_ref, desired_speed, car_speed, cte, dpsi,
                         curv_gate_now, l_eff_ref, desired_delta_rad,
                         car_delta, car_delta_rate, torque_cmd);

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
    // Decision variables: U = [u_plus_0, u_minus_0, ..., u_plus_{N-1}, u_minus_{N-1}]
    //   u_k = u_plus_k − u_minus_k  (both ≥ 0, at most one nonzero at a time)
    //
    // Cost:  Σ_{k=0}^{N-1} [ w_cte * CTE_{k+1}²  +  w_psi * dPsi_{k+1}²
    //                       + w_torque * (u_plus_k − u_minus_k)²
    //                       + w_comp  * (u_plus_k + u_minus_k) ]
    //   w_comp is the L1 complementarity penalty — discourages both being nonzero.
    //
    // Rate constraints (asymmetric, on each component separately):
    //   |u| increasing (0→±1): rate_rising  [norm/s]
    //   |u| decreasing (±1→0): rate_sinking [norm/s]
    //
    // Dynamics (linearised, front-axle rad units), SchedFO2 actuator:
    //   delta_{k+1} = ad_k * delta_k + bd_k * u_k        (speed-scheduled 1st-order)
    //   dPsi_{k+1}  = dPsi_k  − (v * delta_k / L - kappa_k * v) * dt
    //   CTE_{k+1}   = CTE_k   + v * dPsi_k * dt
    //
    // Propagation: each state = constant + G·U_orig  (linear in original u_k)
    //   G vectors remain n-dimensional; expanded on-the-fly for the 2n QP variables.
    // =========================================================================

    double solveMpc(double cte0, double dpsi0, double delta0,
                    double v,    double s_ref,
                    double& out_u_plus, double& out_u_minus)
    {
        const int n  = N_;
        const int nv = 2 * n;  // decision variables: u_plus and u_minus for each step
        if (n < 1) {
            out_u_plus = u_prev_plus_; out_u_minus = u_prev_minus_;
            return u_prev_;
        }

        const double dt          = DT;
        const double w_cte       = get_parameter("weight_cte").as_double();
        const double w_psi       = get_parameter("weight_psi").as_double();
        const double w_t         = get_parameter("weight_torque").as_double();
        const double kus         = get_parameter("understeer_gradient").as_double();   // L_eff = L + Kus*v^2 (P2)
        const double kgate       = get_parameter("disturbance_curvature_gate").as_double();  // P1b

        const double v_fallback = std::max(0.5, v);  // used when no speed profile is available

        // Actuator model is SchedFO2 (1-state) delta[k+1] = ad*delta[k] + bd*u[k];
        // coefficients are recomputed per step at the predicted speed (schedFo2Coeffs).

        // ---- Build QP cost matrices (upper-triangular P, gradient q) --------
        // nv = 2*n variables: [u_plus_0, u_minus_0, ..., u_plus_{n-1}, u_minus_{n-1}]
        const int P_nz = nv * (nv + 1) / 2;
        std::vector<OSQPFloat> P_x(P_nz, 0.0);
        std::vector<OSQPInt>   P_i(P_nz, 0);
        std::vector<OSQPInt>   P_p(nv + 1, 0);
        for (int j = 0; j < nv; j++) P_p[j + 1] = P_p[j] + (j + 1);
        std::vector<OSQPFloat> q_vec(nv, 0.0);

        // P_i: row indices for upper-triangular column-major storage
        for (int j = 0; j < nv; j++)
            for (int i = 0; i <= j; i++)
                P_i[P_p[j] + i] = i;

        // ---- State propagation: x_{k+1} = c_{k+1} + G_{k+1} · U -----------
        double c_torque = u_prev_; // actual rate-limited torque feeding the actuator
        double c_delta = delta0;
        double c_psi   = dpsi0;
        double c_cte   = cte0;

        std::vector<double> G_torque(n, 0.0);  // sensitivity of u_k to each original u_j
        std::vector<double> G_delta(n, 0.0);
        std::vector<double> G_psi  (n, 0.0);
        std::vector<double> G_cte  (n, 0.0);

        // Running arc-length accumulator — advances by v_k*dt each step so the MPC
        // looks ahead using the speed profile rather than a constant measured speed.
        double s_k = s_ref;

        for (int k = 0; k < n; k++) {
            // Per-step speed: from speed profile (curvature-aware) or constant fallback
            const double v_k = path_.hasSpeedProfile()
                ? std::max(get_parameter("v_ref_min_mps").as_double(), path_.vref(s_k))
                : v_fallback;

            // Curvature feedforward at the predicted position for this step
            double kappa = path_.curvature(std::min(s_k, path_.totalLength()));

            // Per-step actuator coefficients at the predicted speed
            double ad_k, bd_k;
            schedFo2Coeffs(v_k * 3.6, ad_k, bd_k);

            // ---- Actuator model: delta_{k+1} = ad_k*delta_k + bd_k*u_ratelimit_k ----
            double c_delta_new = ad_k * c_delta + bd_k * c_torque;
            std::vector<double> G_delta_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_delta_new[j] = ad_k * G_delta[j] + bd_k * G_torque[j];

            // u_k = u_plus_k − u_minus_k; sensitivity to original u_j = δ_{j,k}
            std::vector<double> G_torque_new(n, 0.0);
            G_torque_new[k] = 1.0;
            c_torque = u_prev_;
            G_torque = G_torque_new;

            // dPsi_{k+1} = dPsi_k - (v_k * delta_k / L_eff - kappa_k * v_k) * dt
            // L_eff(v) = L + Kus*v^2 (understeer gradient, P2): the kinematic yaw rate
            // v*delta/L overestimates real turn-in at speed, so the car rides wide in
            // corners. Using L_eff makes the model demand (and the MPC command) the
            // true steer. The path yaw-rate term kappa*v is geometric — left as-is.
            const double l_eff = WHEELBASE + kus * v_k * v_k;
            double c_psi_new = c_psi - (v_k * c_delta / l_eff - kappa * v_k) * dt;
            std::vector<double> G_psi_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_psi_new[j] = G_psi[j] - v_k * dt / l_eff * G_delta[j];

            // CTE_{k+1} = CTE_k + v_k * dPsi_k * dt + w_hat * w_gate * dt
            // Rear-axle reference: steering reaches CTE only via heading (rel-degree 2).
            // w_hat_ is the exogenous disturbance estimate (P1), applied through the
            // per-step curvature gate (P1b) so it acts on straight horizon segments but
            // fades out where the path curves.
            const double w_gate = (kgate > 1e-9)
                ? std::clamp(1.0 - std::abs(kappa) / kgate, 0.0, 1.0) : 1.0;
            double c_cte_new = c_cte + v_k * c_psi * dt + w_hat_ * w_gate * dt;
            std::vector<double> G_cte_new(n, 0.0);
            for (int j = 0; j < n; j++)
                G_cte_new[j] = G_cte[j] + v_k * dt * G_psi[j];

            // Advance state
            c_delta = c_delta_new;  G_delta = G_delta_new;
            c_psi   = c_psi_new;    G_psi   = G_psi_new;
            c_cte   = c_cte_new;    G_cte   = G_cte_new;

            // Accumulate tracking cost from CTE_{k+1} and dPsi_{k+1}.
            // G vectors are n-dimensional (sensitivity to original signed u_j).
            // Expand to 2n QP variables on-the-fly:  G_full[2j]=G[j], G_full[2j+1]=-G[j]
            // Upper-triangular pairs (a,b) in original space → 4 pairs in 2n space:
            //   (+a,+b): val,  (+a,-b): -val,  (-a,+b): -val (b>a only),  (-a,-b): val
            // Terminal weighting (P3): scale the final step's CTE/psi cost to
            // approximate a longer effective horizon without adding decision variables.
            const double tf  = (k == n - 1) ? TERMINAL_WEIGHT_FACTOR : 1.0;
            const double wc_k = w_cte * tf;
            const double wp_k = w_psi * tf;
            for (int a = 0; a < n; a++) {
                for (int b = a; b < n; b++) {
                    double val = static_cast<OSQPFloat>(
                        2.0 * wc_k * G_cte[a] * G_cte[b] +
                        2.0 * wp_k * G_psi[a] * G_psi[b]);
                    P_x[P_p[2*b]   + 2*a  ] += val;   // (+a, +b)
                    P_x[P_p[2*b+1] + 2*a  ] -= val;   // (+a, -b)
                    if (b > a) P_x[P_p[2*b] + 2*a+1] -= val;  // (-a, +b)
                    P_x[P_p[2*b+1] + 2*a+1] += val;   // (-a, -b)
                }
            }
            for (int j = 0; j < n; j++) {
                double dq = 2.0 * wc_k * c_cte * G_cte[j]
                          + 2.0 * wp_k * c_psi * G_psi[j];
                q_vec[2*j]   += static_cast<OSQPFloat>( dq);
                q_vec[2*j+1] += static_cast<OSQPFloat>(-dq);
            }

            // Advance running arc-length for next step
            s_k = std::min(s_k + dt * v_k, path_.totalLength());

            // Torque regularisation: w_t*(u_plus_k - u_minus_k)^2
            // → diagonal 2*w_t at (2k,2k) and (2k+1,2k+1), cross -2*w_t at (2k,2k+1)
            P_x[P_p[2*k]   + 2*k  ] += static_cast<OSQPFloat>(2.0 * w_t);
            P_x[P_p[2*k+1] + 2*k+1] += static_cast<OSQPFloat>(2.0 * w_t);
            P_x[P_p[2*k+1] + 2*k  ] -= static_cast<OSQPFloat>(2.0 * w_t);  // upper-tri cross term

            // Complementarity L1 penalty: w_comp*(u_plus_k + u_minus_k)
            q_vec[2*k]   += static_cast<OSQPFloat>(WEIGHT_COMPLEMENTARITY);
            q_vec[2*k+1] += static_cast<OSQPFloat>(WEIGHT_COMPLEMENTARITY);
        }

        // ---- Build constraint matrix A (asymmetric absolute-value slew + bounds) ----
        // 6 constraints per step k, rows 6k+r:
        //   r=0: u_plus_k  − u_plus_{k-1}  ≤ rate_rising*dt   (|u| increasing)
        //   r=1: u_plus_{k-1}  − u_plus_k  ≤ rate_sinking*dt  (|u| decreasing)
        //   r=2: u_minus_k − u_minus_{k-1} ≤ rate_rising*dt
        //   r=3: u_minus_{k-1} − u_minus_k ≤ rate_sinking*dt
        //   r=4: 0 ≤ u_plus_k  ≤ tlim
        //   r=5: 0 ≤ u_minus_k ≤ tlim
        const int m = 6 * n;
        std::vector<OSQPFloat> A_x;
        std::vector<OSQPInt>   A_i;
        std::vector<OSQPInt>   A_p(nv + 1, 0);
        std::vector<OSQPFloat> l_c(m, static_cast<OSQPFloat>(-OSQP_INFTY));
        std::vector<OSQPFloat> u_c(m, static_cast<OSQPFloat>( OSQP_INFTY));

        // k=0: rate constraints against u_prev_plus_/minus_
        u_c[0] = static_cast<OSQPFloat>( u_prev_plus_  + RATE_RISING  * dt);
        u_c[1] = static_cast<OSQPFloat>(-u_prev_plus_  + RATE_SINKING * dt);
        u_c[2] = static_cast<OSQPFloat>( u_prev_minus_ + RATE_RISING  * dt);
        u_c[3] = static_cast<OSQPFloat>(-u_prev_minus_ + RATE_SINKING * dt);
        l_c[4] = 0.0f;  u_c[4] = static_cast<OSQPFloat>(TORQUE_LIMIT);
        l_c[5] = 0.0f;  u_c[5] = static_cast<OSQPFloat>(TORQUE_LIMIT);
        for (int k = 1; k < n; k++) {
            u_c[6*k+0] = static_cast<OSQPFloat>(RATE_RISING  * dt);
            u_c[6*k+1] = static_cast<OSQPFloat>(RATE_SINKING * dt);
            u_c[6*k+2] = static_cast<OSQPFloat>(RATE_RISING  * dt);
            u_c[6*k+3] = static_cast<OSQPFloat>(RATE_SINKING * dt);
            l_c[6*k+4] = 0.0f;  u_c[6*k+4] = static_cast<OSQPFloat>(TORQUE_LIMIT);
            l_c[6*k+5] = 0.0f;  u_c[6*k+5] = static_cast<OSQPFloat>(TORQUE_LIMIT);
        }

        // CSC column-by-column: column 2k = u_plus_k, column 2k+1 = u_minus_k
        // Row indices within each column must be in ascending order.
        int nz = 0;
        for (int k = 0; k < n; k++) {
            // Column 2k (u_plus_k): rows 6k, 6k+1, 6k+4; if k+1<n: 6(k+1), 6(k+1)+1
            A_p[2*k] = nz;
            A_x.push_back( 1.0f); A_i.push_back(6*k+0);  // rising: +u_plus_k
            A_x.push_back(-1.0f); A_i.push_back(6*k+1);  // sinking: -u_plus_k
            A_x.push_back( 1.0f); A_i.push_back(6*k+4);  // bound: +u_plus_k
            nz += 3;
            if (k + 1 < n) {
                A_x.push_back(-1.0f); A_i.push_back(6*(k+1)+0);  // next rising prev: -u_plus_k
                A_x.push_back( 1.0f); A_i.push_back(6*(k+1)+1);  // next sinking prev: +u_plus_k
                nz += 2;
            }

            // Column 2k+1 (u_minus_k): rows 6k+2, 6k+3, 6k+5; if k+1<n: 6(k+1)+2, 6(k+1)+3
            A_p[2*k+1] = nz;
            A_x.push_back( 1.0f); A_i.push_back(6*k+2);  // rising: +u_minus_k
            A_x.push_back(-1.0f); A_i.push_back(6*k+3);  // sinking: -u_minus_k
            A_x.push_back( 1.0f); A_i.push_back(6*k+5);  // bound: +u_minus_k
            nz += 3;
            if (k + 1 < n) {
                A_x.push_back(-1.0f); A_i.push_back(6*(k+1)+2);
                A_x.push_back( 1.0f); A_i.push_back(6*(k+1)+3);
                nz += 2;
            }
        }
        A_p[nv] = nz;

        // ---- OSQP setup & solve ---------------------------------------------
        OSQPCscMatrix P_csc;
        P_csc.m = nv; P_csc.n = nv; P_csc.nzmax = P_nz; P_csc.nz = -1;
        P_csc.x = P_x.data(); P_csc.i = P_i.data(); P_csc.p = P_p.data();
        P_csc.owned = 0;

        OSQPCscMatrix A_csc;
        A_csc.m = m; A_csc.n = nv;
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
                                 &A_csc, l_c.data(), u_c.data(), m, nv, &settings);
        if (err != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "OSQP setup failed (err=%d); holding previous torque.", static_cast<int>(err));
            out_u_plus = u_prev_plus_; out_u_minus = u_prev_minus_;
            return u_prev_;
        }

        {
            auto t0 = std::chrono::steady_clock::now();
            osqp_solve(osqp_solver_);
            auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now() - t0).count();
            mpc_dbg_.solve_time_us = static_cast<double>(dt_us);
            if (dt_us > 40000) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "OSQP solve overrun: %ld µs (budget 40 ms)", dt_us);
            }
        }
        OSQPInt status_val = osqp_solver_->info->status_val;
        mpc_dbg_.solve_status = static_cast<int>(status_val);
        if (status_val != OSQP_SOLVED && status_val != OSQP_SOLVED_INACCURATE) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "OSQP solve failed (status=%d); holding previous torque.",
                static_cast<int>(status_val));
            out_u_plus = u_prev_plus_; out_u_minus = u_prev_minus_;
            return u_prev_;
        }

        double u_plus0  = std::max(0.0, static_cast<double>(osqp_solver_->solution->x[0]));
        double u_minus0 = std::max(0.0, static_cast<double>(osqp_solver_->solution->x[1]));
        out_u_plus  = u_plus0;
        out_u_minus = u_minus0;

        // Predicted terminal state under the optimal solution. After the propagation
        // loop, c_cte/G_cte and c_psi/G_psi hold the final-step (constant + sensitivity
        // to signed u_j). Reconstruct the horizon-end prediction so the status topic
        // shows whether the MPC itself believes it converges.
        {
            double term_cte = c_cte, term_psi = c_psi;
            for (int j = 0; j < n; j++) {
                double u_j = static_cast<double>(osqp_solver_->solution->x[2*j])
                           - static_cast<double>(osqp_solver_->solution->x[2*j+1]);
                term_cte += G_cte[j] * u_j;
                term_psi += G_psi[j] * u_j;
            }
            mpc_dbg_.pred_terminal_cte = term_cte;
            mpc_dbg_.pred_terminal_psi = term_psi;
        }
        return std::clamp(u_plus0 - u_minus0, -TORQUE_LIMIT, TORQUE_LIMIT);
    }

    // =========================================================================
    // Path loading  (generation + smoothing live in car_control/path.hpp)
    // =========================================================================

    void loadPathCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string& path = msg->data;

        // Empty string = unload path
        if (path.empty()) {
            bool was_following = (state_ == State::FOLLOWING || state_ == State::STOPPING);
            if (was_following) {
                state_ = State::IDLE;
                publishCmd(0.0, 0.0);
                publishStatus(false);
            }
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                path_.clear();
                hint_rear_    = 0;
                u_prev_       = 0.0;
                u_prev_plus_  = 0.0;
                u_prev_minus_ = 0.0;
                w_hat_        = 0.0;
            }
            publishPathVisualization();
            RCLCPP_INFO(get_logger(), "Path unloaded.");
            return;
        }
        // Stop following before swapping the path under the controller
        bool was_following = (state_ == State::FOLLOWING || state_ == State::STOPPING);
        if (was_following) {
            state_ = State::IDLE;
            publishCmd(0.0, 0.0);
            publishStatus(false);
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            loadPathFromCSV(path);
            hint_rear_    = 0;
            u_prev_       = 0.0;
            u_prev_plus_  = 0.0;
            u_prev_minus_ = 0.0;
        }
        publishPathVisualization();
        if (was_following) {
            RCLCPP_INFO(get_logger(), "Path swapped while following — stopped. Re-enable to continue.");
        }
    }

    void loadPathFromCSV(const std::string& filename)
    {
        hint_rear_ = 0;

        int count = path_.loadCSV(filename);   // clears, parses; -1 = open failed
        if (count < 0) {
            RCLCPP_ERROR(get_logger(), "Cannot open path CSV: %s", filename.c_str());
            return;
        }
        if (count < 2) {
            RCLCPP_ERROR(get_logger(),
                "CSV '%s' has fewer than 2 waypoints – no path loaded.",
                filename.c_str());
            return;
        }

        applyPathSmoothing(filename.c_str(), count);
    }

    void applyPathSmoothing(const char* path_label, int waypoint_count)
    {
        if (PATH_SPLINE_KNOT_M > 0.0) {
            path_.smoothSpline(PATH_SPLINE_KNOT_M);
            RCLCPP_INFO(get_logger(),
                "Loaded %s: %d waypoints, %.1f m total (B-spline smoothed, knot spacing %.2f m).",
                path_label, waypoint_count, path_.totalLength(), PATH_SPLINE_KNOT_M);
        } else {
            RCLCPP_INFO(get_logger(),
                "Loaded %s: %d waypoints, %.1f m total (path smoothing disabled).",
                path_label, waypoint_count, path_.totalLength());
        }
        maybeRebuildSpeedProfile();
    }

    void maybeRebuildSpeedProfile()
    {
        if (sched_v_kmh_.empty() || sched_kss_.empty()) return;
        path_.buildSpeedProfile(
            sched_v_kmh_,
            sched_kss_,
            TORQUE_LIMIT,
            get_parameter("desired_speed_mps").as_double(),
            get_parameter("speed_profile_decel_mps2").as_double(),
            get_parameter("speed_profile_accel_mps2").as_double(),
            get_parameter("curvature_speed_margin").as_double(),
            get_parameter("v_ref_min_mps").as_double(),
            get_parameter("curvature_rate_slew_budget_deg_s").as_double() * (M_PI / 180.0));
        auto [kappa_max, v_min, v_max] = path_.speedProfileStats();
        RCLCPP_INFO(get_logger(),
            "Speed profile built: %.2f–%.2f m/s (min–max) over %.1f m. "
            "Max |kappa|=%.4f rad/m (R_min=%.1f m).",
            v_min, v_max, path_.totalLength(),
            kappa_max, kappa_max > 1e-6 ? 1.0 / kappa_max : 9999.0);
    }

    // =========================================================================

    // =========================================================================
    // Publish helpers
    // =========================================================================

    void publishCmd(double accel_cmd, double torque_cmd)
    {
        car_control::msg::DriveCommand cmd;
        cmd.accel  = static_cast<float>(accel_cmd);
        cmd.torque = static_cast<float>(torque_cmd);
        cmd_vel_pub_->publish(cmd);
    }

    void publishStatus(bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        status_pub_->publish(msg);
    }

    // Comprehensive per-tick diagnostics on lateral_mpc/status (MpcStatus).
    void publishMpcStatus(double s_rear, double s_ref, double desired_speed,
                          double car_speed, double cte, double dpsi,
                          double curv_gate_now, double l_eff_ref,
                          double desired_delta_rad, double car_delta,
                          double car_delta_rate, double torque_cmd)
    {
        // Feedforward torque (steady-state torque to hold desired_delta) and the
        // actuator's approx max steer rate, from the SchedFO2 model at this speed.
        double ff_torque = 0.0, steer_rate_limit = 0.0;
        {
            const double vk      = car_speed * 3.6;
            const double kss     = schedInterp(sched_v_kmh_, sched_kss_, vk, 1.0);
            const double kss_rad = kss * (M_PI / 180.0) / STEERING_RATIO;  // rad/torque
            const double tau     = schedInterp(sched_v_kmh_, sched_tau_r_, vk, 0.05);
            if (kss_rad > 1e-9) ff_torque = std::clamp(desired_delta_rad / kss_rad, -1.0, 1.0);
            if (tau > 1e-6)     steer_rate_limit = (kss_rad / tau) * (180.0 / M_PI);
        }

        car_control::msg::MpcStatus st;
        st.header.stamp = this->now();
        st.header.frame_id = "base_link";
        st.state = (state_ == State::FOLLOWING) ? "FOLLOWING"
                 : (state_ == State::STOPPING)  ? "STOPPING" : "IDLE";
        st.progress_m     = s_rear;
        st.path_length_m  = path_.totalLength();
        st.vref_mps       = desired_speed;
        st.v_ego_mps      = car_speed;

        st.cte_m             = cte;
        st.heading_error_deg = dpsi * 180.0 / M_PI;

        st.w_hat_mps         = w_hat_;
        st.disturbance_bias_m = w_hat_ * N_ * DT;
        st.disturbance_gate  = curv_gate_now;
        st.integrator_frozen = mpc_dbg_.integrator_frozen;

        st.kappa_rad_m       = path_.curvature(s_ref);
        st.l_eff_m           = l_eff_ref;
        st.desired_delta_deg = desired_delta_rad * 180.0 / M_PI;
        st.actual_delta_deg  = car_delta * 180.0 / M_PI;
        st.delta_error_deg   = (car_delta - desired_delta_rad) * 180.0 / M_PI;
        st.ff_torque         = ff_torque;
        st.torque_cmd        = torque_cmd;
        st.feedback_torque   = torque_cmd - ff_torque;
        st.torque_saturated  = std::abs(torque_cmd) > 0.97;

        st.steer_rate_deg_s       = car_delta_rate * 180.0 / M_PI;
        st.steer_rate_limit_deg_s = steer_rate_limit;

        st.pred_terminal_cte_m       = mpc_dbg_.pred_terminal_cte;
        st.pred_terminal_heading_deg = mpc_dbg_.pred_terminal_psi * 180.0 / M_PI;
        st.solve_status   = mpc_dbg_.solve_status;
        st.solve_time_us  = mpc_dbg_.solve_time_us;
        st.horizon        = N_;

        pub_status_full_->publish(st);
    }

    // SchedFO2 actuator coefficients at speed v_kmh: delta[k+1] = ad*delta[k] + bd*u[k].
    void schedFo2Coeffs(double v_kmh, double& ad, double& bd) const
    {
        const double tau_v  = schedInterp(sched_v_kmh_, sched_tau_r_, v_kmh, 0.05);
        const double kss_v  = schedInterp(sched_v_kmh_, sched_kss_,   v_kmh, 1.0);
        // Convert K_ss from [sw-deg/torque] to [front-axle rad/torque].
        const double kss_rad = kss_v * (M_PI / 180.0) / STEERING_RATIO;
        ad = std::exp(-DT / tau_v);
        bd = kss_rad * (1.0 - ad);
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
            ps.pose.position.x = x - map_origin_x_;
            ps.pose.position.y = y - map_origin_y_;
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
    double u_prev_       = 0.0;  // effective torque sent last step (ramped)
    double u_prev_plus_  = 0.0;  // positive component of u_prev_
    double u_prev_minus_ = 0.0;  // negative magnitude component of u_prev_
    double w_hat_        = 0.0;  // estimated lateral-disturbance CTE drift [m/s] (offset-free MPC)
    int    N_            = 40;

    // Diagnostics filled by solveMpc, published on lateral_mpc/status each tick
    struct MpcDebug {
        double pred_terminal_cte = 0.0;
        double pred_terminal_psi = 0.0;
        int    solve_status      = 0;
        double solve_time_us     = 0.0;
        bool   integrator_frozen = false;
    } mpc_dbg_;

    // SchedFO2 actuator model breakpoints (loaded from params at startup)
    std::vector<double> sched_v_kmh_;
    std::vector<double> sched_tau_r_;
    std::vector<double> sched_kss_;

    // Map frame origin — subtracted from all published coordinates so values stay
    // near zero for WebGL float32 precision (set from origin_lat/lon params)
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;

    // Path
    Path           path_;
    size_t         hint_rear_  = 0;

    // OSQP workspace (owned; cleaned up in destructor and re-created each solve)
    OSQPSolver* osqp_solver_ = nullptr;

    // ROS 2 handles
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  gnss_pose_sub_;
    rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr   vehicle_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              enable_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            load_path_sub_;

    rclcpp::Publisher<car_control::msg::DriveCommand>::SharedPtr  cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_cte_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_hdg_err_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_desired_delta_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_actual_delta_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_torque_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_progress_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_vref_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_lateral_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_heading_error_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_pf_cmd_vel_;
    rclcpp::Publisher<car_control::msg::MpcStatus>::SharedPtr  pub_status_full_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr auto_enable_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("path_follower_node"),
            "mlockall failed: %s", strerror(errno));
    }

    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
