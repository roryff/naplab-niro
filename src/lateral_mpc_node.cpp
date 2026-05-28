/**
 * @file lateral_mpc_node.cpp
 * @brief Unified lateral MPC for ROS2 autonomous car.
 *
 * Replaces path_follower_node + steering_mpc_node with a single node that
 * performs path following and steering torque control in one OSQP optimisation.
 *
 * 4-state model per step k: [CTE_k, dPsi_k, delta_k, dRate_k]
 *   CTE   — cross-track error [m]  (positive = car right of path)
 *   dPsi  — heading error [rad]    (positive = car pointing right of path)
 *   delta — front-axle steer [rad]
 *   dRate — steering rate [rad/s]
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
#include <nav_msgs/msg/path.hpp>
#include <osqp.h>
#include <sys/mman.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
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

static constexpr double CONTROL_HZ          = 20.0;
static constexpr double DT                  = 1.0 / CONTROL_HZ;  // 0.05 s
static constexpr double WHEELBASE           = 2.79;              // Kia Niro [m]
static constexpr double STEERING_RATIO      = 15.33;             // sw-deg per road-wheel deg (= 460/30; same ratio as in path_follower_node.cpp)
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
     * Positive = vehicle is to the RIGHT of the path.
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

    /**
     * Cubic B-spline least-squares path smoother.
     *
     * Fits independent cubic B-splines x(s) and y(s) with interior knots
     * placed every knot_spacing_m metres along the arc-length.  Solves the
     * normal equations via Cholesky decomposition (Eigen::LLT), then
     * resamples the spline at all original waypoint arc-lengths.
     *
     * This matches the Python LSQUnivariateSpline(k=3) implementation used
     * for offline analysis, with the same 5 m default knot spacing.
     */
    void smoothSpline(double knot_spacing_m)
    {
        const int n = static_cast<int>(wpts_.size());
        if (n < 4) return;

        const double L = s_.back();
        if (L < knot_spacing_m) return;

        // --- Build interior knot vector ---
        std::vector<double> interior_knots;
        for (double t = knot_spacing_m; t < L - knot_spacing_m * 0.5; t += knot_spacing_m)
            interior_knots.push_back(t);

        // Full knot vector: degree k=3 requires k+1 repeated knots at each end.
        const int k = 3;
        std::vector<double> knots;
        for (int i = 0; i <= k; i++) knots.push_back(0.0);
        for (double t : interior_knots) knots.push_back(t);
        for (int i = 0; i <= k; i++) knots.push_back(L);

        const int m = static_cast<int>(knots.size());
        const int nc = m - k - 1;  // number of B-spline control points

        // --- Cox–de Boor basis evaluation ---
        // Returns row vector of B_{i,k}(t) for i = 0 … nc-1.
        auto bsplineBasis = [&](double t) -> Eigen::RowVectorXd {
            // Clamp to valid range
            t = std::clamp(t, 0.0, L);
            // For t == L, push into last span
            if (t >= L) t = L - 1e-10;

            Eigen::RowVectorXd B = Eigen::RowVectorXd::Zero(nc);

            // Degree-0 basis: indicator for knot span
            std::vector<double> d(m - 1, 0.0);
            for (int i = 0; i < m - 1; i++) {
                if (t >= knots[i] && t < knots[i+1])
                    d[i] = 1.0;
            }

            // De Boor recursion from degree 1 to k
            for (int deg = 1; deg <= k; deg++) {
                std::vector<double> d2(m - 1 - deg, 0.0);
                for (int i = 0; i < static_cast<int>(d2.size()); i++) {
                    double left = 0.0, right = 0.0;
                    double dl = knots[i + deg] - knots[i];
                    double dr = knots[i + deg + 1] - knots[i + 1];
                    if (dl > 1e-12) left  = (t - knots[i])           / dl * d[i];
                    if (dr > 1e-12) right = (knots[i+deg+1] - t)     / dr * d[i+1];
                    d2[i] = left + right;
                }
                d.resize(d2.size());
                d = d2;
            }

            for (int i = 0; i < nc; i++) B(i) = d[i];
            return B;
        };

        // --- Build least-squares system A (n x nc) ---
        Eigen::MatrixXd A(n, nc);
        for (int i = 0; i < n; i++)
            A.row(i) = bsplineBasis(s_[i]);

        Eigen::VectorXd fx(n), fy(n);
        for (int i = 0; i < n; i++) {
            fx(i) = wpts_[i].first;
            fy(i) = wpts_[i].second;
        }

        // Normal equations: (A^T A) c = A^T f
        Eigen::MatrixXd ATA = A.transpose() * A;
        Eigen::LLT<Eigen::MatrixXd> llt(ATA);
        if (llt.info() != Eigen::Success) return;  // singular — skip smoothing

        Eigen::VectorXd cx = llt.solve(A.transpose() * fx);
        Eigen::VectorXd cy = llt.solve(A.transpose() * fy);

        // --- Resample spline at original arc-lengths ---
        for (int i = 0; i < n; i++) {
            Eigen::RowVectorXd B = bsplineBasis(s_[i]);
            wpts_[i].first  = B.dot(cx);
            wpts_[i].second = B.dot(cy);
        }

        // Rebuild arc-length table
        s_[0] = 0.0;
        for (int i = 1; i < n; i++) {
            double dx = wpts_[i].first  - wpts_[i-1].first;
            double dy = wpts_[i].second - wpts_[i-1].second;
            s_[i] = s_[i-1] + std::hypot(dx, dy);
        }
    }

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
    enum class ReferencePoint { RearAxle, FrontAxle };

public:
    LateralMpcNode()
    : Node("lateral_mpc_node"),
      state_(State::IDLE),
      path_start_time_(this->now()),
      hint_front_(0)
    {
        timer_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // ---- Parameters --------------------------------------------------------
        declare_parameter("desired_speed_mps", 4.0);
        declare_parameter("stop_distance",     3.0);
        declare_parameter("horizon",           40);
        declare_parameter("weight_cte",        2.0);   // [1/m²]
        declare_parameter("weight_psi",        1.0);   // [1/rad²]
        declare_parameter("weight_torque",     0.1);
        declare_parameter("tau_r",             0.78);  // actuator time constant [s]
        declare_parameter("gain_r",            36.0);  // sw-deg/s per unit torque
        declare_parameter("rate_rising",  1.033);   // |u| increasing (0→±1) [norm/s]
        declare_parameter("rate_sinking", 1.833);   // |u| decreasing (±1→0) [norm/s]
        declare_parameter("weight_complementarity", 1.0);  // L1 penalty on u_plus + u_minus
        declare_parameter("torque_limit",      1.0);
        declare_parameter("kp_speed",          0.3);
        declare_parameter<std::string>("path_csv_file", "");
        declare_parameter<bool>("auto_enable", false);
        declare_parameter("weight_delta", 0.0);  // delta reference tracking weight; 0=disabled
        declare_parameter<std::string>("reference_point", "rear_axle");
        declare_parameter("path_spline_knot_m", 5.0);
        declare_parameter<bool>  ("use_sched_fo2", false);
        declare_parameter<std::vector<double>>("sched_v_kmh", std::vector<double>{});
        declare_parameter<std::vector<double>>("sched_tau_r", std::vector<double>{});
        declare_parameter<std::vector<double>>("sched_kss",   std::vector<double>{});

        N_ = static_cast<int>(std::max(1L, std::min(get_parameter("horizon").as_int(), (int64_t)64)));
        reference_point_ = loadReferencePoint();

        // ---- Actuator model ----------------------------------------------------
        use_sched_fo2_ = get_parameter("use_sched_fo2").as_bool();
        if (use_sched_fo2_) {
            sched_v_kmh_ = get_parameter("sched_v_kmh").as_double_array();
            sched_tau_r_ = get_parameter("sched_tau_r").as_double_array();
            sched_kss_   = get_parameter("sched_kss").as_double_array();
            RCLCPP_INFO(get_logger(),
                "sched_fo2 actuator model: %zu breakpoints, tau_r %.2f..%.2f s",
                sched_v_kmh_.size(),
                sched_tau_r_.empty() ? 0.0 : sched_tau_r_.front(),
                sched_tau_r_.empty() ? 0.0 : sched_tau_r_.back());
        } else {
            RCLCPP_INFO(get_logger(), "Fixed actuator model: tau_r=%.2f gain_r=%.1f",
                get_parameter("tau_r").as_double(), get_parameter("gain_r").as_double());
        }

        // ---- Subscriptions -----------------------------------------------------
        gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "gnss/pose", 10,
            std::bind(&LateralMpcNode::gnssPoseCallback, this, std::placeholders::_1));

        vehicle_state_sub_ = create_subscription<car_control::msg::VehicleState>(
            "vehicle/state", rclcpp::SensorDataQoS(),
            std::bind(&LateralMpcNode::vehicleStateCallback, this, std::placeholders::_1));

        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "enable_path_following", 10,
            std::bind(&LateralMpcNode::enableCallback, this, std::placeholders::_1));

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
        pub_lateral_error_ = create_publisher<std_msgs::msg::Float64>("lateral_error",                 10);
        pub_heading_error_ = create_publisher<std_msgs::msg::Float64>("heading_error",                 10);
        pub_pf_cmd_vel_    = create_publisher<geometry_msgs::msg::Twist>("path_follower/cmd_vel",       10);

        // ---- Load-path subscription (runtime path switching) -----------------
        load_path_sub_ = create_subscription<std_msgs::msg::String>(
            "/lateral_mpc/load_path", rclcpp::QoS(1),
            std::bind(&LateralMpcNode::loadPathCallback, this, std::placeholders::_1));

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
            std::bind(&LateralMpcNode::controlLoop, this),
            timer_cb_group_);

        auto_enable_ = get_parameter("auto_enable").as_bool();

        RCLCPP_INFO(get_logger(),
            "LateralMpcNode ready. Path: %.1f m  N=%d  ref=%s  spline_knot=%.2f m  weight_delta=%.2f. "
            "Publish 'true' on ~/enable_path_following to start.",
            path_.totalLength(), N_, referencePointName(reference_point_),
            get_parameter("path_spline_knot_m").as_double(),
            get_parameter("weight_delta").as_double());

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
            hint_rear_       = 0;
            u_prev_          = 0.0;
            u_prev_plus_     = 0.0;
            u_prev_minus_    = 0.0;
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

        // --- Project front axle onto path ------------------------------------
        double rear_x  = car_x;
        double rear_y  = car_y;
        double front_x = rear_x + WHEELBASE * std::cos(car_heading);
        double front_y = rear_y + WHEELBASE * std::sin(car_heading);
        double s_rear  = path_.findClosest(rear_x, rear_y, hint_rear_);
        double s_front = path_.findClosest(front_x, front_y, hint_front_);

        const bool use_front_reference = (reference_point_ == ReferencePoint::FrontAxle);
        const double ref_x = use_front_reference ? front_x : rear_x;
        const double ref_y = use_front_reference ? front_y : rear_y;
        const double s_ref = use_front_reference ? s_front : s_rear;

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

        // --- Initial state for MPC -------------------------------------------
        double cte  = path_.crossTrackError(ref_x, ref_y, s_ref);
        double dpsi = path_.headingError(s_ref, car_heading);

        // Curvature-based desired steer angle (feedforward reference for debug only)
        double desired_delta_rad = path_.curvature(s_ref) * WHEELBASE;

        // --- Solve MPC -------------------------------------------------------
        double torque_cmd = 0.0;
        if (state_ == State::FOLLOWING) {
            double out_plus = 0.0, out_minus = 0.0;
            torque_cmd = ramp * solveMpc(cte, dpsi, car_delta, car_delta_rate,
                                         car_speed, s_ref, out_plus, out_minus);
            u_prev_plus_  = ramp * out_plus;
            u_prev_minus_ = ramp * out_minus;
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
    // Dynamics (linearised, front-axle rad units):
    //   dRate_{k+1} = a_rate * dRate_k + b_rate * u_k
    //   delta_{k+1} = delta_k + dt * dRate_{k+1}
    //   dPsi_{k+1}  = dPsi_k  − (v * delta_k / L - kappa_k * v) * dt
    //   CTE_{k+1}   = CTE_k   + v * dPsi_k * dt
    //
    // Propagation: each state = constant + G·U_orig  (linear in original u_k)
    //   G vectors remain n-dimensional; expanded on-the-fly for the 2n QP variables.
    // =========================================================================

    double solveMpc(double cte0, double dpsi0, double delta0, double drate0,
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
        const double w_d         = get_parameter("weight_delta").as_double();
        const double w_comp      = get_parameter("weight_complementarity").as_double();
        const double rate_rising  = get_parameter("rate_rising").as_double();
        const double rate_sinking = get_parameter("rate_sinking").as_double();
        const double tlim        = get_parameter("torque_limit").as_double();

        const double v_eff = std::max(0.5, v);  // guard against division by zero at low speed
        const double v_kmh = v * 3.6;

        // ---- Actuator model coefficients ----------------------------------------
        // sched_fo2 (1-state): delta[k+1] = ad*delta[k] + bd*u[k]
        // Legacy 2-state:      dRate[k+1] = a_rate*dRate[k] + b_rate*u[k]
        //                      delta[k+1] = delta[k] + dt*dRate[k+1]
        double ad = 0.0, bd = 0.0;
        double a_rate = 0.0, b_rate = 0.0;
        if (use_sched_fo2_) {
            const double tau_v  = schedInterp(sched_v_kmh_, sched_tau_r_, v_kmh, 0.05);
            const double kss_v  = schedInterp(sched_v_kmh_, sched_kss_,   v_kmh, 1.0);
            // Convert K_ss from [sw-deg/torque] to [front-axle rad/torque]
            const double kss_rad = kss_v * (M_PI / 180.0) / STEERING_RATIO;
            ad = std::exp(-dt / tau_v);
            bd = kss_rad * (1.0 - ad);
        } else {
            const double tau_r     = get_parameter("tau_r").as_double();
            const double gain_r    = get_parameter("gain_r").as_double();
            const double alpha     = dt / (tau_r + dt);
            a_rate = 1.0 - alpha;
            // gain_r: 36 sw-deg/s/unit → rad/s/unit at front axle
            const double gain_r_rad = gain_r * (M_PI / 180.0) / STEERING_RATIO;
            b_rate = alpha * gain_r_rad;
        }

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
        double c_rate  = drate0;  // only used by legacy 2-state model
        double c_torque = u_prev_; // actual rate-limited torque (for sched_fo2)
        double c_delta = delta0;
        double c_psi   = dpsi0;
        double c_cte   = cte0;

        std::vector<double> G_rate  (n, 0.0);  // only used by legacy 2-state model
        std::vector<double> G_torque(n, 0.0);  // sched_fo2: sensitivity of u_k to each u_j (original)
        std::vector<double> G_delta(n, 0.0);
        std::vector<double> G_psi  (n, 0.0);
        std::vector<double> G_cte  (n, 0.0);

        for (int k = 0; k < n; k++) {
            // Path curvature feedforward at predicted vehicle position for step k
            double s_k   = std::min(s_ref + k * dt * v_eff, path_.totalLength());
            double kappa = path_.curvature(s_k);

            // ---- Actuator model: compute delta_{k+1} ----------------------------
            double c_delta_new;
            std::vector<double> G_delta_new(n, 0.0);
            if (use_sched_fo2_) {
                // sched_fo2 model: delta[k+1] = ad*delta[k] + bd*u_ratelimit[k]
                // where u_ratelimit[k] is the actual torque after rate limiting (not raw u[k])
                // The rate limiting is enforced by OSQP constraints, so we model with the constrained torque.
                c_delta_new = ad * c_delta + bd * c_torque;
                for (int j = 0; j < n; j++)
                    G_delta_new[j] = ad * G_delta[j] + bd * G_torque[j];

                // u_k = u_plus_k − u_minus_k; sensitivity to original u_j = δ_{j,k}
                double c_torque_new = u_prev_;
                std::vector<double> G_torque_new(n, 0.0);
                G_torque_new[k] = 1.0;
                c_torque = c_torque_new;
                G_torque = G_torque_new;
            } else {
                // dRate_{k+1} = a_rate * dRate_k + b_rate * u_k
                double c_rate_new = a_rate * c_rate;
                std::vector<double> G_rate_new(n, 0.0);
                for (int j = 0; j < n; j++)
                    G_rate_new[j] = a_rate * G_rate[j] + (j == k ? b_rate : 0.0);
                // delta_{k+1} = delta_k + dt * dRate_{k+1}
                c_delta_new = c_delta + dt * c_rate_new;
                for (int j = 0; j < n; j++)
                    G_delta_new[j] = G_delta[j] + dt * G_rate_new[j];
                c_rate = c_rate_new;
                G_rate = G_rate_new;
            }

            // dPsi_{k+1} = dPsi_k - (v * delta_k / L - kappa_k * v) * dt
            // Positive steer left increases car heading, decreasing dPsi = path_heading - car_heading
            double c_psi_new = c_psi - (v_eff * c_delta / WHEELBASE - kappa * v_eff) * dt;
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
            c_delta = c_delta_new;  G_delta = G_delta_new;
            c_psi   = c_psi_new;    G_psi   = G_psi_new;
            c_cte   = c_cte_new;    G_cte   = G_cte_new;

            // Accumulate tracking cost from CTE_{k+1} and dPsi_{k+1}.
            // G vectors are n-dimensional (sensitivity to original signed u_j).
            // Expand to 2n QP variables on-the-fly:  G_full[2j]=G[j], G_full[2j+1]=-G[j]
            // Upper-triangular pairs (a,b) in original space → 4 pairs in 2n space:
            //   (+a,+b): val,  (+a,-b): -val,  (-a,+b): -val (b>a only),  (-a,-b): val
            for (int a = 0; a < n; a++) {
                for (int b = a; b < n; b++) {
                    double val = static_cast<OSQPFloat>(
                        2.0 * w_cte * G_cte[a] * G_cte[b] +
                        2.0 * w_psi * G_psi[a] * G_psi[b]);
                    P_x[P_p[2*b]   + 2*a  ] += val;   // (+a, +b)
                    P_x[P_p[2*b+1] + 2*a  ] -= val;   // (+a, -b)
                    if (b > a) P_x[P_p[2*b] + 2*a+1] -= val;  // (-a, +b)
                    P_x[P_p[2*b+1] + 2*a+1] += val;   // (-a, -b)
                }
            }
            for (int j = 0; j < n; j++) {
                double dq = 2.0 * w_cte * c_cte * G_cte[j]
                          + 2.0 * w_psi * c_psi * G_psi[j];
                q_vec[2*j]   += static_cast<OSQPFloat>( dq);
                q_vec[2*j+1] += static_cast<OSQPFloat>(-dq);
            }

            // Delta reference tracking cost: w_d * (delta_{k+1} - kappa_{k+1}*L)^2
            // Guides MPC to reach the curvature-required steer angle, preventing pre-steer.
            if (w_d > 0.0) {
                double s_next = std::min(s_ref + (k + 1) * dt * v_eff, path_.totalLength());
                double delta_ref = path_.curvature(s_next) * WHEELBASE;
                double d = c_delta - delta_ref;
                for (int a = 0; a < n; a++) {
                    for (int b = a; b < n; b++) {
                        double dval = static_cast<OSQPFloat>(2.0 * w_d * G_delta[a] * G_delta[b]);
                        P_x[P_p[2*b]   + 2*a  ] += dval;
                        P_x[P_p[2*b+1] + 2*a  ] -= dval;
                        if (b > a) P_x[P_p[2*b] + 2*a+1] -= dval;
                        P_x[P_p[2*b+1] + 2*a+1] += dval;
                    }
                }
                for (int j = 0; j < n; j++) {
                    double dq = 2.0 * w_d * d * G_delta[j];
                    q_vec[2*j]   += static_cast<OSQPFloat>( dq);
                    q_vec[2*j+1] += static_cast<OSQPFloat>(-dq);
                }
            }

            // Torque regularisation: w_t*(u_plus_k - u_minus_k)^2
            // → diagonal 2*w_t at (2k,2k) and (2k+1,2k+1), cross -2*w_t at (2k,2k+1)
            P_x[P_p[2*k]   + 2*k  ] += static_cast<OSQPFloat>(2.0 * w_t);
            P_x[P_p[2*k+1] + 2*k+1] += static_cast<OSQPFloat>(2.0 * w_t);
            P_x[P_p[2*k+1] + 2*k  ] -= static_cast<OSQPFloat>(2.0 * w_t);  // upper-tri cross term

            // Complementarity L1 penalty: w_comp*(u_plus_k + u_minus_k)
            q_vec[2*k]   += static_cast<OSQPFloat>(w_comp);
            q_vec[2*k+1] += static_cast<OSQPFloat>(w_comp);
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
        u_c[0] = static_cast<OSQPFloat>( u_prev_plus_  + rate_rising  * dt);
        u_c[1] = static_cast<OSQPFloat>(-u_prev_plus_  + rate_sinking * dt);
        u_c[2] = static_cast<OSQPFloat>( u_prev_minus_ + rate_rising  * dt);
        u_c[3] = static_cast<OSQPFloat>(-u_prev_minus_ + rate_sinking * dt);
        l_c[4] = 0.0f;  u_c[4] = static_cast<OSQPFloat>(tlim);
        l_c[5] = 0.0f;  u_c[5] = static_cast<OSQPFloat>(tlim);
        for (int k = 1; k < n; k++) {
            u_c[6*k+0] = static_cast<OSQPFloat>(rate_rising  * dt);
            u_c[6*k+1] = static_cast<OSQPFloat>(rate_sinking * dt);
            u_c[6*k+2] = static_cast<OSQPFloat>(rate_rising  * dt);
            u_c[6*k+3] = static_cast<OSQPFloat>(rate_sinking * dt);
            l_c[6*k+4] = 0.0f;  u_c[6*k+4] = static_cast<OSQPFloat>(tlim);
            l_c[6*k+5] = 0.0f;  u_c[6*k+5] = static_cast<OSQPFloat>(tlim);
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
            if (dt_us > 40000) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "OSQP solve overrun: %ld µs (budget 40 ms)", dt_us);
            }
        }
        OSQPInt status_val = osqp_solver_->info->status_val;
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
        return std::clamp(u_plus0 - u_minus0, -tlim, tlim);
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
        hint_front_ = 0;
        hint_rear_  = 0;

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

        applyPathSmoothing("Sinusoidal path", n_tot + 1);
    }

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
                hint_front_   = 0;
                hint_rear_    = 0;
                u_prev_       = 0.0;
                u_prev_plus_  = 0.0;
                u_prev_minus_ = 0.0;
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
            hint_front_   = 0;
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
        path_.clear();
        hint_front_ = 0;
        hint_rear_  = 0;

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

        applyPathSmoothing(filename.c_str(), count);
    }

    ReferencePoint loadReferencePoint()
    {
        const std::string value = get_parameter("reference_point").as_string();
        if (value == "rear" || value == "rear_axle" || value == "cg") {
            return ReferencePoint::RearAxle;
        }
        if (value == "front" || value == "front_axle") {
            return ReferencePoint::FrontAxle;
        }
        RCLCPP_WARN(get_logger(),
            "Unknown reference_point='%s'; defaulting to rear_axle.",
            value.c_str());
        return ReferencePoint::RearAxle;
    }

    const char* referencePointName(ReferencePoint point) const
    {
        return (point == ReferencePoint::FrontAxle) ? "front_axle" : "rear_axle";
    }

    void applyPathSmoothing(const char* path_label, int waypoint_count)
    {
        const double knot_m = get_parameter("path_spline_knot_m").as_double();
        if (knot_m > 0.0) {
            path_.smoothSpline(knot_m);
            RCLCPP_INFO(get_logger(),
                "Loaded %s: %d waypoints, %.1f m total (B-spline smoothed, knot spacing %.2f m).",
                path_label, waypoint_count, path_.totalLength(), knot_m);
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Loaded %s: %d waypoints, %.1f m total (path smoothing disabled).",
            path_label, waypoint_count, path_.totalLength());
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
    double u_prev_       = 0.0;  // effective torque sent last step (ramped)
    double u_prev_plus_  = 0.0;  // positive component of u_prev_
    double u_prev_minus_ = 0.0;  // negative magnitude component of u_prev_
    int    N_            = 40;

    // sched_fo2 actuator model (loaded from sched_fo2_model_path if provided)
    bool use_sched_fo2_ = false;
    std::vector<double> sched_v_kmh_;
    std::vector<double> sched_tau_r_;
    std::vector<double> sched_kss_;

    // Path
    Path           path_;
    ReferencePoint reference_point_ = ReferencePoint::RearAxle;
    size_t         hint_front_ = 0;
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_lateral_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       pub_heading_error_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_pf_cmd_vel_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr auto_enable_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("lateral_mpc_node"),
            "mlockall failed: %s", strerror(errno));
    }

    auto node = std::make_shared<LateralMpcNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
