/**
 * @file sysid_node.cpp
 * @brief Steering actuator system identification node.
 * 
 *ros2 run car_control sysid_node \
    --ros-args -p test_mode:=steady_state \
               -p amplitude:=1.0 \
               -p desired_speed_kmh:=20.0 \
               -p step_hold_s:=20.0
10,15,20,25,30,35,40,45,50
 * Generates open-loop torque excitation signals (chirp, PRBS, step sequence, rate-limit
 * characterisation) while holding target speed with a P-controller.
 * All signals are recorded via rosbag — record /vehicle/state, /cmd_vel,
 * /sysid/torque_cmd, /sysid/status.
 *
 * Safety gates
 * ------------
 *   - Only outputs torque when `vehicle/state.lat_active == true`.
 *   - Waits until speed is within speed_tol_mps of desired before starting excitation.
 *
 * Subscriptions
 * -------------
 *   vehicle/state   (car_control/VehicleState)          – speed, steer angle, torques
 *   gnss/gyro       (geometry_msgs/Vector3Stamped)       – yaw rate [rad/s] (optional)
 *
 * Publications
 * ------------
 *   cmd_vel         (car_control/DriveCommand)            – accel [-1,1], torque [-1,1]
 *   sysid/torque_cmd  (std_msgs/Float64)                 – excitation signal (debug)
 *   sysid/status      (std_msgs/String)                  – IDLE/RUNNING/DONE/ABORTED
 *
 * Test modes
 * ----------
 *   prbs         – random binary signal: alternates +/-amplitude with random hold times.
 *                  Broadband excitation that works correctly with the panda rate limiter —
 *                  signal is always fully settled at +/-A so rate-limited transitions are
 *                  brief transients, not distorted waveforms. Use car_output_torque as the
 *                  MATLAB input u. Default min/max hold: 0.5–3.0 s (covers 0.15–2 Hz).
 *   chirp        – linear frequency sweep. Only use at amplitude <= 0.30 to stay below
 *                  the panda rate limiter and keep sinusoids undistorted.
 *   step         – one-shot: -A hold, 0 hold, +A hold, 0 hold (step_hold_s each)
 *   steady_state – long saturated hold to find true steering steady-state:
 *                  1 s baseline → +A held for step_hold_s → 2 s neutral →
 *                  -A held for step_hold_s → 2 s neutral.
 *                  Use step_hold_s >= 20 s so the wheel fully converges before the
 *                  operator takes over. lat_active dropping always terminates safely.
 *   ratelimit    – fast full-step to characterise panda safety rate limits
 *   idle         – zero torque (for baseline / debug)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include "car_control/msg/drive_command.hpp"
#include "car_control/msg/vehicle_state.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <string>

// Control rate matches all other nodes in this package.
static constexpr double CONTROL_HZ = 20.0;
static constexpr double DT         = 1.0 / CONTROL_HZ;

class SysidNode : public rclcpp::Node
{
    enum class State { IDLE, RUNNING, DONE, ABORTED };

public:
    SysidNode() : Node("sysid_node"), state_(State::IDLE), t_elapsed_(0.0),
                  u_prev_(0.0), prbs_current_val_(1.0), prbs_switch_t_(0.0),
                  rng_(std::random_device{}()), gyro_z_(0.0), has_gyro_(false)
    {
        // ---- Parameters --------------------------------------------------------
        declare_parameter<std::string>("test_mode",        "chirp");
        declare_parameter<double>     ("amplitude",        0.70);
        declare_parameter<double>     ("freq_start",       0.20);
        declare_parameter<double>     ("freq_end",         2.0);
        declare_parameter<double>     ("duration_s",       60.0);
        declare_parameter<double>     ("desired_speed_kmh", 18.0);
        declare_parameter<double>     ("kp_speed",         0.3);
        declare_parameter<double>     ("speed_tol_kmh",    2.0);
        declare_parameter<double>     ("step_hold_s",      20.0);
        declare_parameter<double>     ("prbs_min_hold_s",  0.5);
        declare_parameter<double>     ("prbs_max_hold_s",  3.0);

        test_mode_      = get_parameter("test_mode").as_string();
        amplitude_      = get_parameter("amplitude").as_double();
        freq_start_     = get_parameter("freq_start").as_double();
        freq_end_       = get_parameter("freq_end").as_double();
        duration_s_     = get_parameter("duration_s").as_double();
        desired_speed_  = get_parameter("desired_speed_kmh").as_double() / 3.6;
        kp_speed_       = get_parameter("kp_speed").as_double();
        speed_tol_mps_  = get_parameter("speed_tol_kmh").as_double() / 3.6;
        step_hold_s_    = get_parameter("step_hold_s").as_double();
        prbs_min_hold_  = get_parameter("prbs_min_hold_s").as_double();
        prbs_max_hold_  = get_parameter("prbs_max_hold_s").as_double();
        // Auto-set duration from step_hold_s for timed modes
        if (test_mode_ == "step")         duration_s_ = 4.0 * step_hold_s_;
        if (test_mode_ == "steady_state") duration_s_ = 2.0 * step_hold_s_ + 5.0;

        // ---- Subscriptions -----------------------------------------------------
        sub_state_ = create_subscription<car_control::msg::VehicleState>(
            "vehicle/state", rclcpp::SensorDataQoS(),
            [this](const car_control::msg::VehicleState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                v_ego_kmh_         = static_cast<double>(msg->v_ego);
                steering_angle_    = static_cast<double>(msg->steering_angle_deg);
                car_output_torque_ = static_cast<double>(msg->car_output_torque);
                actuators_torque_  = static_cast<double>(msg->actuators_torque);
                lat_active_        = msg->lat_active;
                state_received_    = true;
            });

        sub_gyro_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "gnss/gyro", 10,
            [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                gyro_z_    = msg->vector.z;
                has_gyro_  = true;
            });

        // ---- Publications ------------------------------------------------------
        pub_cmd_vel_  = create_publisher<car_control::msg::DriveCommand>("cmd_vel", 10);
        pub_torque_   = create_publisher<std_msgs::msg::Float64>("sysid/torque_cmd", 10);
        pub_status_   = create_publisher<std_msgs::msg::String>("sysid/status", 10);

        // ---- Timer -------------------------------------------------------------
        timer_ = create_wall_timer(
            std::chrono::duration<double>(DT),
            std::bind(&SysidNode::controlLoop, this));

        RCLCPP_INFO(get_logger(),
            "SysidNode ready.  mode=%s  A=%.2f  f=%.2f-%.2f Hz  dur=%.0fs  "
            "target_speed=%.1f km/h",
            test_mode_.c_str(), amplitude_,
            freq_start_, freq_end_, duration_s_, desired_speed_ * 3.6);
        RCLCPP_INFO(get_logger(),
            "Waiting for lat_active=true to begin excitation.");

        publishStatus("IDLE");
    }

    ~SysidNode() = default;

private:
    // =========================================================================
    // Control loop (20 Hz)
    // =========================================================================

    void controlLoop()
    {
        double v_ego_kmh;
        bool lat_active, state_received;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            v_ego_kmh      = v_ego_kmh_;
            lat_active     = lat_active_;
            state_received = state_received_;
        }

        const double v_mps = v_ego_kmh / 3.6;

        // ---- State machine ---------------------------------------------------
        if (state_ == State::DONE || state_ == State::ABORTED) {
            publishBrake();
            return;
        }

        if (!state_received) {
            publishBrake();
            return;
        }

        if (state_ == State::RUNNING) {
            if (!lat_active) {
                RCLCPP_WARN(get_logger(),
                    "lat_active dropped — pausing excitation.");
                publishHold(v_mps);
                return;
            }
        }

        // ---- Wait for lat_active and target speed ----------------------------
        if (state_ == State::IDLE) {
            if (!lat_active) {
                publishBrake();
                return;
            }
            if (std::abs(v_mps - desired_speed_) > speed_tol_mps_) {
                publishHold(v_mps);
                return;
            }
            RCLCPP_INFO(get_logger(),
                "Speed reached (%.1f m/s) — starting excitation.", v_mps);
            state_     = State::RUNNING;
            t_elapsed_ = 0.0;
            u_prev_    = 0.0;
            publishStatus("RUNNING");
        }

        // ---- Generate excitation signal -------------------------------------
        double u_torque = computeExcitation(t_elapsed_);
        t_elapsed_ += DT;

        if (t_elapsed_ >= duration_s_) {
            RCLCPP_INFO(get_logger(), "Excitation complete (%.0f s).", duration_s_);
            state_ = State::DONE;
            publishStatus("DONE");
            publishBrake();
            return;
        }

        u_prev_ = u_torque;

        // ---- Speed P-controller ---------------------------------------------
        double speed_err  = desired_speed_ - v_mps;
        double accel_cmd  = std::clamp(kp_speed_ * speed_err, -1.0, 1.0);

        // ---- Publish --------------------------------------------------------
        car_control::msg::DriveCommand cmd;
        cmd.accel  = static_cast<float>(accel_cmd);
        cmd.torque = static_cast<float>(u_torque);
        pub_cmd_vel_->publish(cmd);

        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = u_torque;
        pub_torque_->publish(torque_msg);
    }

    // =========================================================================
    // Excitation signal generators
    // =========================================================================

    double computeExcitation(double t)
    {
        if (test_mode_ == "prbs") {
            return prbs(t);
        } else if (test_mode_ == "chirp") {
            return chirp(t);
        } else if (test_mode_ == "step") {
            return stepSequence(t);
        } else if (test_mode_ == "steady_state") {
            return steadyState(t);
        } else if (test_mode_ == "ratelimit") {
            return rateLimitTest(t);
        }
        // "idle" or unknown
        return 0.0;
    }

    /**
     * Random Binary Signal (RBS): alternates between +amplitude and -amplitude,
     * holding each level for a random duration drawn uniformly from
     * [prbs_min_hold_s, prbs_max_hold_s].
     *
     * Why this works with the panda rate limiter:
     *   - The signal is always at +A or -A (fully settled), never mid-swing.
     *   - Rate-limited transitions are brief ramps (< 1/rate_up = ~1 s for full step).
     *   - As long as prbs_min_hold_s >> 1/rate_up, the settled segments dominate and
     *     car_output_torque is a clean binary signal with short ramp transitions.
     *   - Use car_output_torque (not u_cmd) as the MATLAB input u.
     */
    double prbs(double t)
    {
        if (t >= prbs_switch_t_) {
            prbs_current_val_ = -prbs_current_val_;
            std::uniform_real_distribution<double> dist(prbs_min_hold_, prbs_max_hold_);
            prbs_switch_t_ = t + dist(rng_);
        }
        return amplitude_ * prbs_current_val_;
    }

    /**
     * Linear chirp: u(t) = A · sin(2π · φ(t))
     *   φ(t) = (f_start + (f_end - f_start) · t / (2 · duration)) · t
     *
     * Starts at 0 Hz component offset so the first quarter-wave is positive,
     * giving a clean initial transient for delay estimation.
     */
    double chirp(double t)
    {
        const double T    = duration_s_;
        const double f0   = freq_start_;
        const double f1   = freq_end_;
        double phase = 2.0 * M_PI * (f0 * t + (f1 - f0) * t * t / (2.0 * T));
        return amplitude_ * std::sin(phase);
    }

    /**
     * One-shot step sequence: +amplitude → 0 → -amplitude → 0, each step_hold_s long.
     */
    double stepSequence(double t)
    {
        if      (t < step_hold_s_)           return  -amplitude_;
        else if (t < 2.0 * step_hold_s_)     return  0.0;
        else if (t < 3.0 * step_hold_s_)     return amplitude_;
        else                                  return  0.0;
    }

    /**
     * Saturated hold sequence — finds the true steady-state steering angle.
     *
     * Timeline (step_hold_s = H):
     *   0–1 s        →  0          (baseline, wheel centred)
     *   1–(1+H) s    → +amplitude  (hold until wheel converges — watch steering_angle_deg)
     *   (1+H)–(3+H)  →  0          (recovery)
     *   (3+H)–(3+2H) → −amplitude  (opposite direction)
     *   (3+2H)–(5+2H)→  0          (trailing rest; DONE triggered by duration timer)
     *
     * Set step_hold_s >= 20 s.  The operator can safely take over at any time
     * (lat_active drop stops excitation immediately).
     */
    double steadyState(double t)
    {
        if      (t < 1.0)                          return  0.0;
        else if (t < 1.0 + step_hold_s_)           return  amplitude_;
        else if (t < 3.0 + step_hold_s_)           return  0.0;
        else if (t < 3.0 + 2.0 * step_hold_s_)    return -amplitude_;
        else                                        return  0.0;
    }

    /**
     * Rate-limit characterisation sequence (duration_s should be ~30 s):
     *   0–2 s   →  0 (baseline)
     *   2–7 s   → +1.0 (full positive step, observe rise)
     *   7–12 s  →  0   (observe fall)
     *   12–17 s → −1.0 (full negative step)
     *   17–22 s →  0   (observe fall)
     *   22–30 s →  0   (rest)
     */
    double rateLimitTest(double t)
    {
        if      (t <  2.0)  return  0.0;
        else if (t <  7.0)  return  1.0;
        else if (t < 12.0)  return  0.0;
        else if (t < 17.0)  return -1.0;
        else                return  0.0;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    void abort()
    {
        state_ = State::ABORTED;
        publishStatus("ABORTED");
        publishBrake();
    }

    void publishBrake()
    {
        car_control::msg::DriveCommand msg;
        msg.accel  = -1.0f;
        msg.torque =  0.0f;
        pub_cmd_vel_->publish(msg);
    }

    /** Hold torque at zero, run speed controller only. */
    void publishHold(double v_mps)
    {
        double accel_cmd = std::clamp(kp_speed_ * (desired_speed_ - v_mps), -1.0, 1.0);
        car_control::msg::DriveCommand msg;
        msg.accel  = static_cast<float>(accel_cmd);
        msg.torque =  0.0f;
        pub_cmd_vel_->publish(msg);
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::msg::String msg;
        msg.data = s;
        pub_status_->publish(msg);
        RCLCPP_INFO(get_logger(), "Status: %s", s.c_str());
    }

    // =========================================================================
    // Members
    // =========================================================================

    State  state_;
    double t_elapsed_;
    double u_prev_;

    // Parameters
    std::string test_mode_;
    double amplitude_;
    double freq_start_;
    double freq_end_;
    double duration_s_;
    double desired_speed_;
    double kp_speed_;
    double speed_tol_mps_;
    double step_hold_s_;
    double prbs_min_hold_;
    double prbs_max_hold_;

    // PRBS state
    double prbs_current_val_;
    double prbs_switch_t_;
    std::mt19937 rng_;

    // Sensor state (protected by mutex_)
    std::mutex mutex_;
    double v_ego_kmh_         = 0.0;
    double steering_angle_    = 0.0;
    double car_output_torque_ = 0.0;
    double actuators_torque_  = 0.0;
    double gyro_z_            = 0.0;
    bool   lat_active_        = false;
    bool   state_received_    = false;
    bool   has_gyro_          = false;

    // ROS handles
    rclcpp::Subscription<car_control::msg::VehicleState>::SharedPtr          sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr       sub_gyro_;
    rclcpp::Publisher<car_control::msg::DriveCommand>::SharedPtr              pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                      pub_torque_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                       pub_status_;
    rclcpp::TimerBase::SharedPtr                                              timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SysidNode>());
    rclcpp::shutdown();
    return 0;
}
