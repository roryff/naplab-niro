# Parsed UBX-ESF-STATUS message
std_msgs/Header header

uint8  fusion_mode     # 0=INIT 1=FUSION 2=SUSPENDED 3=DISABLED
string fusion_label    # human-readable fusion mode string

string wt_label        # wheel-tick init state: off | initializing | initialized
string mnt_alg_label   # mount-alignment init state
string ins_label       # INS init state
string imu_label       # IMU init state

uint8             num_sens
EsfSensor[]       sensors
