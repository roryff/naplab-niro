/**
 * ROS2 C++ MPC node for steering torque control.
 * Loads integrator model from YAML, builds QP (same as Python torque_controller.py),
 * solves with OSQP, publishes first torque. Can be copied into an existing ROS2 project.
 */

#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <osqp.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "car_control/msg/vehicle_state.hpp"

namespace {

constexpr double kDefaultTauR = 0.2;
constexpr double kDefaultGainR = 40.0;
constexpr double kDefaultLeak = 0.0;
constexpr double kDefaultU0 = 0.0;
constexpr double kDefaultRateClip = 500.0;
constexpr double kDefaultMaxAngleDeg = 450.0;
constexpr double kDefaultDelayS = 0.0;
constexpr double kDefaultDt = 0.1;
constexpr int kDefaultHorizon = 10;
constexpr double kDefaultRateUp = 3.1 / 3.0;
constexpr double kDefaultRateDown = 5.5 / 3.0;
constexpr double kDefaultTorqueLimit  = 1.0;
constexpr double kDefaultKpSpeed     = 0.3;   // accel cmd per (m/s) speed error
constexpr double kDefaultMaxSpeedMps = 15.0;  // clamp desired_speed input
constexpr double kDefaultWeightAngle = 1.0;
constexpr double kDefaultWeightTorque = 0.01;

struct IntegratorParams {
  double tau_r = kDefaultTauR;
  double gain_r = kDefaultGainR;
  double leak = kDefaultLeak;
  double u0 = kDefaultU0;
  double rate_clip = kDefaultRateClip;
  double max_angle_deg = kDefaultMaxAngleDeg;
  double delay_s = kDefaultDelayS;
};

struct ControllerParams {
  double dt = kDefaultDt;
  int horizon = kDefaultHorizon;
  double rate_up = kDefaultRateUp;
  double rate_down = kDefaultRateDown;
  double torque_limit = kDefaultTorqueLimit;
  double weight_angle = kDefaultWeightAngle;
  double weight_torque = kDefaultWeightTorque;
};

// Simple YAML parser for "key: value" lines (no nesting).
bool loadConfig(const std::string& path, IntegratorParams& ip, ControllerParams& cp) {
  std::ifstream f(path);
  if (!f.is_open()) return false;
  std::string line;
  while (std::getline(f, line)) {
    auto pos = line.find('#');
    if (pos != std::string::npos) line = line.substr(0, pos);
    pos = line.find(':');
    if (pos == std::string::npos) continue;
    std::string key = line.substr(0, pos);
    std::string val = line.substr(pos + 1);
    auto trim = [](std::string& s) {
      size_t i = 0; while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) i++;
      s = s.substr(i);
      while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
    };
    trim(key); trim(val);
    if (key == "tau_r") ip.tau_r = std::stod(val);
    else if (key == "gain_r") ip.gain_r = std::stod(val);
    else if (key == "leak") ip.leak = std::stod(val);
    else if (key == "u0") ip.u0 = std::stod(val);
    else if (key == "rate_clip") ip.rate_clip = std::stod(val);
    else if (key == "max_angle_deg") ip.max_angle_deg = std::stod(val);
    else if (key == "delay_s") ip.delay_s = std::stod(val);
    else if (key == "dt") cp.dt = std::stod(val);
    else if (key == "horizon") cp.horizon = std::stoi(val);
    else if (key == "rate_up") cp.rate_up = std::stod(val);
    else if (key == "rate_down") cp.rate_down = std::stod(val);
    else if (key == "torque_limit") cp.torque_limit = std::stod(val);
    else if (key == "weight_angle") cp.weight_angle = std::stod(val);
    else if (key == "weight_torque") cp.weight_torque = std::stod(val);
  }
  return true;
}

}  // namespace

class SteeringMpcNode : public rclcpp::Node {
 public:
  SteeringMpcNode() : Node("steering_mpc_node") {
    declare_parameter<std::string>("model_config_path", "");
    declare_parameter<double>("dt", kDefaultDt);
    declare_parameter<int>("horizon", kDefaultHorizon);
    declare_parameter<double>("rate_up", kDefaultRateUp);
    declare_parameter<double>("rate_down", kDefaultRateDown);
    declare_parameter<double>("torque_limit",   kDefaultTorqueLimit);
    declare_parameter<double>("weight_angle",    kDefaultWeightAngle);
    declare_parameter<double>("weight_torque",   kDefaultWeightTorque);
    declare_parameter<double>("kp_speed",        kDefaultKpSpeed);
    declare_parameter<double>("max_speed_mps",   kDefaultMaxSpeedMps);
    declare_parameter<double>("desired_speed_mps", 0.0);

    std::string config_path = get_parameter("model_config_path").as_string();
    if (!config_path.empty()) {
      if (!loadConfig(config_path, integrator_, ctrl_)) {
        RCLCPP_ERROR(get_logger(), "Failed to load model config: %s", config_path.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded model from %s", config_path.c_str());
      }
    }
    ctrl_.dt = get_parameter("dt").as_double();
    ctrl_.horizon = get_parameter("horizon").as_int();
    ctrl_.rate_up = get_parameter("rate_up").as_double();
    ctrl_.rate_down = get_parameter("rate_down").as_double();
    ctrl_.torque_limit = get_parameter("torque_limit").as_double();
    ctrl_.weight_angle = get_parameter("weight_angle").as_double();
    ctrl_.weight_torque = get_parameter("weight_torque").as_double();    kp_speed_          = get_parameter("kp_speed").as_double();
    max_speed_mps_     = get_parameter("max_speed_mps").as_double();
    // Initialize desired speed from parameter so accel runs immediately at startup
    desired_speed_mps_ = get_parameter("desired_speed_mps").as_double();
    N_ = std::max(1, std::min(ctrl_.horizon, 64));

    // Subscribe to path_follower output: angular.z = desired front-axle angle [rad]
    // (remapped to path_follower/cmd_vel in launch file)
    sub_path_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "path_follower/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          // Convert front-axle steer [rad] → steering wheel [deg]
          // Kia Niro: 460 deg lock-to-lock ↔ 0.5236 rad (30 deg) front axle
          constexpr double MAX_FRONT_RAD   = 0.5236;
          constexpr double MAX_WHEEL_DEG   = 460.0;
          desired_angle_ = msg->angular.z * (MAX_WHEEL_DEG / MAX_FRONT_RAD);
          // Speed comes from desired_speed_mps parameter, not from path_follower
        });

    // Gate MPC output on path following status
    sub_pf_status_ = create_subscription<std_msgs::msg::Bool>(
        "/path_following_status", rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          path_following_active_ = msg->data;
          if (!msg->data) {
            // Reset integrator state so we don't wind up while idle
            u_prev_       = 0.0;
            current_rate_ = 0.0;
          }
          RCLCPP_INFO(get_logger(), "Path following %s",
              msg->data ? "ACTIVE — MPC running" : "INACTIVE — MPC output suppressed");
        });

    // Subscribe to vehicle/state for steering wheel angle [deg] and speed [km/h]
    sub_vehicle_state_ = create_subscription<car_control::msg::VehicleState>(
        "vehicle/state", 10,
        [this](const car_control::msg::VehicleState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          double new_angle = static_cast<double>(msg->steering_angle_deg);
          double t = rclcpp::Clock().now().seconds();
          if (last_angle_time_ > 0) {
            double dt = t - last_angle_time_;
            if (dt > 0 && dt < 1.0) {
              current_rate_ = (new_angle - current_angle_) / dt;
            }
          }
          current_angle_   = new_angle;
          last_angle_time_ = t;
          // v_ego is km/h → convert to m/s
          vehicle_speed_mps_ = static_cast<double>(msg->v_ego) / 3.6;
        });

    // Publish cmd_vel to comma_node: angular.z = steering torque [-1,1], linear.x = accel cmd [-1,1]
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Debug topics for rosbag / rqt_plot evaluation
    pub_dbg_desired_angle_  = create_publisher<std_msgs::msg::Float64>("mpc/desired_angle_deg",  10);
    pub_dbg_actual_angle_   = create_publisher<std_msgs::msg::Float64>("mpc/actual_angle_deg",   10);
    pub_dbg_angle_error_    = create_publisher<std_msgs::msg::Float64>("mpc/angle_error_deg",    10);
    pub_dbg_actual_rate_    = create_publisher<std_msgs::msg::Float64>("mpc/actual_rate_deg_s",  10);
    pub_dbg_torque_cmd_     = create_publisher<std_msgs::msg::Float64>("mpc/torque_cmd",         10);
    pub_dbg_accel_cmd_      = create_publisher<std_msgs::msg::Float64>("mpc/accel_cmd",          10);
    pub_dbg_desired_speed_  = create_publisher<std_msgs::msg::Float64>("mpc/desired_speed_mps",  10);
    pub_dbg_actual_speed_   = create_publisher<std_msgs::msg::Float64>("mpc/actual_speed_mps",   10);
    pub_dbg_speed_error_    = create_publisher<std_msgs::msg::Float64>("mpc/speed_error_mps",    10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(ctrl_.dt),
        [this]() { runMpcStep(); });

    RCLCPP_INFO(get_logger(),
      "SteeringMpcNode ready  sub: path_follower/cmd_vel + vehicle/state  pub: cmd_vel");
  }

  ~SteeringMpcNode() {
    if (osqp_workspace_) {
      osqp_cleanup(osqp_workspace_);
      osqp_workspace_ = nullptr;
    }
  }

 private:
  void runMpcStep() {
    double angle_deg, rate_deg_s, desired, u_prev, desired_speed, vehicle_speed;
    bool active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active        = path_following_active_;
      angle_deg     = current_angle_;
      rate_deg_s    = current_rate_;
      desired       = desired_angle_;
      u_prev        = u_prev_;
      desired_speed = std::clamp(desired_speed_mps_, -max_speed_mps_, max_speed_mps_);
      vehicle_speed = vehicle_speed_mps_;
    }

    // Do not run MPC when path following is inactive — hold brake
    if (!active) {
      geometry_msgs::msg::Twist brake;
      brake.linear.x  = -1.0;  // full brake
      brake.angular.z =  0.0;
      pub_cmd_vel_->publish(brake);
      return;
    }

    double u_cmd = solveMpc(angle_deg, rate_deg_s, desired, u_prev);
    u_cmd = std::max(-ctrl_.torque_limit, std::min(ctrl_.torque_limit, u_cmd));
    u_prev_ = u_cmd;

    // Speed P-controller → acceleration command [-1, 1]
    double speed_err  = desired_speed - vehicle_speed;
    double accel_cmd  = std::clamp(kp_speed_ * speed_err, -1.0, 1.0);

    // Publish to cmd_vel – comma_node axes[0]=accel, axes[1]=steering torque
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = accel_cmd;  // closed-loop accel [-1, 1]
    cmd.angular.z = u_cmd;      // MPC steering torque [-1, 1]
    pub_cmd_vel_->publish(cmd);

    // Publish debug topics
    auto f64 = [](double v) { std_msgs::msg::Float64 m; m.data = v; return m; };
    pub_dbg_desired_angle_ ->publish(f64(desired));              // MPC target [deg sw]
    pub_dbg_actual_angle_  ->publish(f64(angle_deg));            // Actual sw angle [deg]
    pub_dbg_angle_error_   ->publish(f64(desired - angle_deg));  // Error [deg] (> 0 = need more left)
    pub_dbg_actual_rate_   ->publish(f64(rate_deg_s));           // Measured sw rate [deg/s]
    pub_dbg_torque_cmd_    ->publish(f64(u_cmd));                // MPC torque output [-1, 1]
    pub_dbg_accel_cmd_     ->publish(f64(accel_cmd));            // Speed P-ctrl accel [-1, 1]
    pub_dbg_desired_speed_ ->publish(f64(desired_speed));        // Target speed [m/s]
    pub_dbg_actual_speed_  ->publish(f64(vehicle_speed));        // Actual speed [m/s]
    pub_dbg_speed_error_   ->publish(f64(speed_err));            // Speed error [m/s]

    RCLCPP_DEBUG(get_logger(),
      "MPC: desired_sw=%.1f° current=%.1f° torque=%.3f  speed_des=%.2f actual=%.2f accel=%.3f",
      desired, angle_deg, u_cmd, desired_speed, vehicle_speed, accel_cmd);
  }

  double solveMpc(double angle_deg, double rate_deg_s, double desired_angle, double u_prev) {
    const int n = N_;
    if (n < 1) return u_prev;

    const double dt = ctrl_.dt;
    const double tau_r = integrator_.tau_r;
    const double gain_r = integrator_.gain_r;
    const double leak = integrator_.leak;
    const double alpha = dt / (tau_r + dt);
    const double a_rate = 1.0 - alpha;
    const double b_rate = alpha * gain_r;
    const double a_angle = 1.0 - dt * leak;
    const double b_angle = dt;
    const double wa = ctrl_.weight_angle;
    const double wt = ctrl_.weight_torque;
    const double rate_up = ctrl_.rate_up;
    const double rate_down = ctrl_.rate_down;
    const double tlim = ctrl_.torque_limit;

    // Cost = 0.5 x' P x + q' x, x = [U_0 .. U_{n-1}]. Match Python: angle_{k+1} = a_angle*angle_k + b_angle*rate_{k+1}, rate_{k+1} = a_rate*rate_k + b_rate*U_k.
    // Track constant term and gradient: angle_pred = c_ang + G_ang'*U, rate_pred = c_rate + G_rate'*U.
    double c_ang = angle_deg;
    double c_rate = rate_deg_s;
    std::vector<double> G_ang(n, 0.0), G_rate(n, 0.0);

    const int P_nz = n * (n + 1) / 2;
    std::vector<OSQPFloat> P_x(P_nz, 0.0);
    std::vector<OSQPInt>   P_i(P_nz, 0);
    std::vector<OSQPInt>   P_p(n + 1, 0);
    for (int j = 0; j < n; j++) P_p[j + 1] = P_p[j] + (j + 1);
    std::vector<OSQPFloat> q(n, 0.0);

    const double ref = desired_angle;
    for (int k = 0; k < n; k++) {
      double c_rate_new = a_rate * c_rate;
      std::vector<double> G_rate_new(n, 0.0);
      for (int j = 0; j < n; j++)
        G_rate_new[j] = a_rate * G_rate[j] + (j == k ? b_rate : 0.0);

      double c_ang_new = a_angle * c_ang + b_angle * c_rate_new;
      std::vector<double> G_ang_new(n, 0.0);
      for (int j = 0; j < n; j++)
        G_ang_new[j] = a_angle * G_ang[j] + b_angle * G_rate_new[j];

      c_ang = c_ang_new;
      c_rate = c_rate_new;
      G_ang = G_ang_new;
      G_rate = G_rate_new;

      double d = c_ang - ref;
      for (int j = 0; j < n; j++)
        q[j] += 2.0 * wa * d * G_ang[j];
      for (int i = 0; i < n; i++)
        for (int j = i; j < n; j++)
          P_x[P_p[j] + i] += 2.0 * wa * G_ang[i] * G_ang[j];
      P_x[P_p[k] + k] += 2.0 * wt;
    }

    for (int j = 0; j < n; j++)
      for (int i = 0; i <= j; i++)
        P_i[P_p[j] + i] = i;

    // Constraints l <= A*x <= u. Row 4k: U_k <= u_prev_k + rate_up*dt; 4k+1: U_k >= u_prev_k - rate_down*dt; 4k+2: U_k <= tlim; 4k+3: U_k >= -tlim. u_prev_0 = u_prev, u_prev_k = U_{k-1}.
    const int m = 4 * n;
    std::vector<OSQPFloat> A_x;
    std::vector<OSQPInt>   A_i;
    std::vector<OSQPInt>   A_p(n + 1, 0);
    std::vector<OSQPFloat> l(m, -OSQP_INFTY);
    std::vector<OSQPFloat> u(m, OSQP_INFTY);

    u[0] = u_prev + rate_up * dt;
    u[1] = -u_prev + rate_down * dt;
    u[2] = tlim;
    u[3] = tlim;
    for (int k = 1; k < n; k++) {
      u[4 * k] = rate_up * dt;
      u[4 * k + 1] = rate_down * dt;
      u[4 * k + 2] = tlim;
      u[4 * k + 3] = tlim;
    }

    int nz = 0;
    for (int j = 0; j < n; j++) {
      A_p[j] = nz;
      if (j == 0) {
        A_x.insert(A_x.end(), {1.0, -1.0, 1.0, -1.0});
        A_i.insert(A_i.end(), {0, 1, 2, 3});
        nz += 4;
      } else {
        A_x.push_back(-1.0); A_i.push_back(4 * (j - 1));
        A_x.push_back(1.0);  A_i.push_back(4 * (j - 1) + 1);
        A_x.push_back(1.0);  A_i.push_back(4 * j);
        A_x.push_back(-1.0); A_i.push_back(4 * j + 1);
        A_x.push_back(1.0);  A_i.push_back(4 * j + 2);
        A_x.push_back(-1.0); A_i.push_back(4 * j + 3);
        nz += 6;
      }
    }
    A_p[n] = nz;

    OSQPCscMatrix P_csc;
    P_csc.m = n; P_csc.n = n; P_csc.nzmax = P_nz; P_csc.nz = -1;
    P_csc.x = P_x.data(); P_csc.i = P_i.data(); P_csc.p = P_p.data();
    P_csc.owned = 0;

    OSQPCscMatrix A_csc;
    A_csc.m = m; A_csc.n = n; A_csc.nzmax = static_cast<OSQPInt>(A_x.size());
    A_csc.nz = -1;
    A_csc.x = A_x.data(); A_csc.i = A_i.data(); A_csc.p = A_p.data();
    A_csc.owned = 0;

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;

    if (osqp_workspace_) {
      osqp_cleanup(osqp_workspace_);
      osqp_workspace_ = nullptr;
    }
    OSQPInt err = osqp_setup(&osqp_workspace_, &P_csc, q.data(), &A_csc, l.data(), u.data(), m, n, &settings);
    if (err != 0) return u_prev;
    osqp_solve(osqp_workspace_);
    OSQPInt status = osqp_workspace_->info->status_val;
    if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE) {
      return u_prev;
    }
    double u0 = osqp_workspace_->solution->x[0];
    return std::max(-tlim, std::min(tlim, u0));
  }

  IntegratorParams integrator_;
  ControllerParams ctrl_;
  int N_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      sub_path_cmd_;
  rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            sub_pf_status_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_cmd_vel_;
  // Debug publishers (rosbag / rqt_plot)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_desired_angle_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_actual_angle_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_angle_error_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_actual_rate_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_torque_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_accel_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_desired_speed_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_actual_speed_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dbg_speed_error_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  double current_angle_     = 0.0;
  double current_rate_      = 0.0;
  double desired_angle_     = 0.0;
  double desired_speed_mps_ = 0.0;
  double vehicle_speed_mps_ = 0.0;
  double last_angle_time_   = -1.0;
  double u_prev_            = 0.0;
  double kp_speed_          = kDefaultKpSpeed;
  double max_speed_mps_     = kDefaultMaxSpeedMps;
  bool   path_following_active_ = false;
  OSQPSolver* osqp_workspace_ = nullptr;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringMpcNode>());
  rclcpp::shutdown();
  return 0;
}
