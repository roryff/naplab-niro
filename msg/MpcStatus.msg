# Comprehensive lateral-MPC diagnostics — one message per control tick (20 Hz).
# Lets us see what each term contributes so corner CTE can be attributed and tuned.

std_msgs/Header header

string  state                  # "IDLE" | "FOLLOWING" | "STOPPING"
float64 progress_m             # arc-length along path
float64 path_length_m
float64 vref_mps               # commanded speed (profile + lookahead)
float64 v_ego_mps              # measured speed

# --- Longitudinal: feedforward vs feedback breakdown ---
float64 accel_cmd              # total longitudinal command sent [-1,1]

# --- Tracking errors (the things we want at zero) ---
float64 cte_m                  # cross-track error (rear axle), + = car right of path
float64 heading_error_deg      # + = car pointing right of path

# CTE integrator
float64 cte_integrator_mps             
float64 cte_integrator_bias_m    

# --- Steering: feedforward vs feedback breakdown ---
float64 kappa_rad_m            # path curvature at current projection
float64 l_eff_m                # effective wheelbase L + Kus*v^2 (understeer model)
float64 desired_delta_deg      # FF steer = kappa * L_eff  (front axle)
float64 actual_delta_deg       # measured front-axle steer
float64 delta_error_deg        # actual - desired  (+ = over-steering FF)
float64 torque_cmd             # total torque sent [-1,1]
bool    torque_saturated       # |torque_cmd| > 0.97

# --- Actuator rate (is it rate-limited here?) ---
float64 steer_rate_deg_s       # measured d(delta)/dt

# --- MPC internal prediction (does the MPC THINK it will converge?) ---
float64 pred_terminal_cte_m    # predicted CTE at end of horizon under optimal solution
float64 pred_terminal_heading_deg
int32   solve_status           # OSQP status_val
float64 solve_time_us
int32   horizon

# --- Friction-aware steering (stiction fix) ---
float64 friction_comp          # signed breakaway assist added on top of the MPC torque
float64 dist_delta_deg         # yaw-rate observer: steer-equivalent disturbance (camber, slip)
