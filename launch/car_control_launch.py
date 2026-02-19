from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # GNSS node - connects directly to u-blox receiver
        Node(
            package='car_control',
            executable='gnss_node',
            name='gnss_node',
            output='screen',
            parameters=[{
                'host': '192.168.10.61',  # u-blox receiver IP address
                'port': 7799,              # u-blox TCP port
                'reconnect_interval_sec': 5.0,
                'auto_set_origin': True,  # Automatically set origin at first fix
                # 'origin_lat': 63.4305,  # Uncomment to manually set origin (Trondheim example)
                # 'origin_lon': 10.3951,
                # 'origin_alt': 0.0,
            }],
            # Enable DEBUG logging to see calibration status
            ros_arguments=['--log-level', 'gnss_node:=DEBUG']
        ),
        
        # Comma.ai CAN interface node
        Node(
            package='car_control',
            executable='comma_node',
            name='comma_node',
            output='screen'
        ),
        
        # Path follower node
        Node(
            package='car_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[{
                'wheelbase': 2.7,              # Kia Niro wheelbase [m]
                'max_steering_angle_deg': 30.0,
                'max_steering_wheel_angle_deg': 460.0,
                'lookahead_distance': 5.0,
                'stanley_k_e': 0.5,            # Cross-track error gain
                'stanley_k_v': 1.0,            # Softening term
                'max_speed_kmh': 30.0,
                'min_recording_distance': 2.0  # Min distance between recorded points [m]
            }]
        ),
    ])


