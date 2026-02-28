from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('car_control')

    # ---- TCP / ADB connection --------------------------------------------------
    use_tcp_tunnel  = LaunchConfiguration('use_tcp_tunnel',  default='true')
    tcp_listen_port = LaunchConfiguration('tcp_listen_port', default='3000')
    adb_host        = LaunchConfiguration('adb_host',        default='127.0.0.1')
    adb_port        = LaunchConfiguration('adb_port',        default='5555')

    # ---- Path follower / replay ------------------------------------------------
    path_csv_file    = LaunchConfiguration('path_csv_file',    default='')
    desired_speed    = LaunchConfiguration('desired_speed_mps', default='4.0')

    # ---- Steering MPC ----------------------------------------------------------
    model_config     = LaunchConfiguration(
        'model_config_path',
        default=PathJoinSubstitution([pkg_share, 'config', 'integrator_model.yaml']))

    # ---- Fixed ENU origin (Naplab parking area) --------------------------------
    origin_lat = LaunchConfiguration('origin_lat', default='63.4391')
    origin_lon = LaunchConfiguration('origin_lon', default='10.4128')
    origin_alt = LaunchConfiguration('origin_alt', default='3.2')

    return LaunchDescription([
        # ---- Argument declarations ---------------------------------------------
        DeclareLaunchArgument('use_tcp_tunnel',  default_value='true',
            description='Listen for comma device over TCP (true) instead of ADB (false)'),
        DeclareLaunchArgument('tcp_listen_port', default_value='3000',
            description='Port to listen on in TCP tunnel mode'),
        DeclareLaunchArgument('adb_host',        default_value='127.0.0.1',
            description='ADB device host (use_tcp_tunnel=false only)'),
        DeclareLaunchArgument('adb_port',        default_value='5555',
            description='ADB device port (use_tcp_tunnel=false only)'),

        DeclareLaunchArgument('path_csv_file', default_value='',
            description='Path to recorded drive CSV for replay (empty = sinusoidal test path)'),
        DeclareLaunchArgument('desired_speed_mps', default_value='4.0',
            description='Desired replay speed [m/s]'),
        DeclareLaunchArgument('model_config_path', default_value='',
            description='Path to integrator_model.yaml for steering MPC (empty = use defaults)'),

        DeclareLaunchArgument('origin_lat', default_value='63.4391',
            description='ENU origin latitude  (fixed so all drives share the same frame)'),
        DeclareLaunchArgument('origin_lon', default_value='10.4128',
            description='ENU origin longitude'),
        DeclareLaunchArgument('origin_alt', default_value='3.2',
            description='ENU origin altitude [m]'),
        Node(
            package='car_control',
            executable='comma_node',
            name='comma_node',
            output='screen',
            parameters=[{
                'use_tcp_tunnel': use_tcp_tunnel,
                'tcp_listen_port': tcp_listen_port,
                'adb_host': adb_host,
                'adb_port': adb_port,
            }],
        ),

        # ---- gnss_node ---------------------------------------------------------
        Node(
            package='car_control',
            executable='gnss_node',
            name='gnss_node',
            output='screen',
            parameters=[{
                'auto_set_origin': False,
                'origin_lat': origin_lat,
                'origin_lon': origin_lon,
                'origin_alt': origin_alt,
            }],
        ),

        # ---- path_follower_node ------------------------------------------------
        #  Publishes cmd_vel (front-axle steer angle [rad] + desired speed [m/s])
        #  Remapped to path_follower/cmd_vel so steering_mpc_node can intercept it.
        Node(
            package='car_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[{
                'path_csv_file':    path_csv_file,
                'desired_speed_mps': desired_speed,
                'auto_enable':      True,
            }],
            remappings=[
                ('cmd_vel', 'path_follower/cmd_vel'),
            ],
        ),

        # ---- steering_mpc_node -------------------------------------------------
        #  Bridges:  path_follower/cmd_vel + vehicle/state  →  cmd_vel (torque + speed)
        #  comma_node reads cmd_vel.angular.z as steering torque [-1, 1]
        Node(
            package='car_control',
            executable='steering_mpc_node',
            name='steering_mpc_node',
            output='screen',
            parameters=[{
                'model_config_path': model_config,
                'desired_speed_mps': desired_speed,
                'dt':      0.05,   # 20 Hz – match path follower rate
                'horizon': 10,
            }],
        ),

        # ---- dashboard_server --------------------------------------------------
        Node(
            package='car_control',
            executable='dashboard_server.py',
            name='dashboard_server',
            output='screen',
        ),
    ])
