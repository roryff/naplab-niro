# Unified vehicle sensor state from comma device
# All sensor data published at 50 Hz

std_msgs/Header header             # ROS timestamp (now()); see also timestamp below

# Timestamp
int64 timestamp                    # nanoseconds since system boot (Comma device clock, not ROS time)

# Kinematics
float32 v_ego                      # Vehicle speed in km/h
float32 steering_angle_deg         # Steering wheel angle in degrees
float32 rear_wheel_speed_left      # Left rear wheel speed in m/s
float32 rear_wheel_speed_right     # Right rear wheel speed in m/s

# Actuators
float32 actuators_accel            # Acceleration command
float32 actuators_torque           # torque command

# Vehicle outputs (after panda safety)
float32 car_output_accel           # Actual acceleration output
float32 car_output_torque          # Actual torque applied

# Control states
bool lat_active                    # Lateral control active
bool long_active                   # Longitudinal control active
