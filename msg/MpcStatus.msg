# Comprehensive lateral-MPC diagnostics — one message per control tick (20 Hz).
# Lets us see what each term contributes so corner CTE can be attributed and tuned.

std_msgs/Header header

string  state                  # "IDLE" | "FOLLOWING" | "STOPPING"
float64 progress_m             # arc-length along path
float64 path_length_m
float64 vref_mps               # commanded speed (profile + lookahead)
float64 v_ego_mps              # measured speed

# --- Tracking errors (the things we want at zero) ---
float64 cte_m                  # cross-track error (rear axle), + = car right of path
float64 heading_error_deg      # + = car pointing right of path

# --- Disturbance estimator (offset-free / integral, P1) ---
float64 w_hat_mps              # estimated lateral drift [m/s] fed into prediction
float64 disturbance_bias_m     # = w_hat * N * dt : predicted CTE shift the integral injects
float64 disturbance_gate       # curvature gate [0..1]: how much w_hat is estimated/applied here (1=straight, 0=corner)
bool    integrator_frozen      # true when anti-windup froze it (torque saturated)

# --- Steering: feedforward vs feedback breakdown ---
float64 kappa_rad_m            # path curvature at current projection
float64 l_eff_m                # effective wheelbase L + Kus*v^2 (understeer model)
float64 desired_delta_deg      # FF steer = kappa * L_eff  (front axle)
float64 actual_delta_deg       # measured front-axle steer
float64 delta_error_deg        # actual - desired  (+ = over-steering FF)
float64 ff_torque              # steady-state torque to hold desired_delta (actuator model)
float64 torque_cmd             # total torque sent [-1,1]
float64 feedback_torque        # torque_cmd - ff_torque  (the correction part)
bool    torque_saturated       # |torque_cmd| > 0.97

# --- Actuator rate (is it rate-limited here?) ---
float64 steer_rate_deg_s       # measured d(delta)/dt
float64 steer_rate_limit_deg_s # approx max achievable rate at this speed (model)

# --- MPC internal prediction (does the MPC THINK it will converge?) ---
float64 pred_terminal_cte_m    # predicted CTE at end of horizon under optimal solution
float64 pred_terminal_heading_deg
int32   solve_status           # OSQP status_val
float64 solve_time_us
int32   horizon
