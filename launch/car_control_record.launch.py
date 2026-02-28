from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_tcp_tunnel  = LaunchConfiguration('use_tcp_tunnel',  default='true')
    tcp_listen_port = LaunchConfiguration('tcp_listen_port', default='3000')
    adb_host        = LaunchConfiguration('adb_host',        default='127.0.0.1')
    adb_port        = LaunchConfiguration('adb_port',        default='5555')
    origin_lat      = LaunchConfiguration('origin_lat',      default='63.4391')
    origin_lon      = LaunchConfiguration('origin_lon',      default='10.4128')
    origin_alt      = LaunchConfiguration('origin_alt',      default='3.2')

    return LaunchDescription([
        DeclareLaunchArgument('use_tcp_tunnel',  default_value='true',
            description='Listen for comma device over TCP (true) instead of ADB (false)'),
        DeclareLaunchArgument('tcp_listen_port', default_value='3000',
            description='Port to listen on in TCP tunnel mode'),
        DeclareLaunchArgument('adb_host',        default_value='127.0.0.1',
            description='ADB device host (use_tcp_tunnel=false only)'),
        DeclareLaunchArgument('adb_port',        default_value='5555',
            description='ADB device port (use_tcp_tunnel=false only)'),
        DeclareLaunchArgument('origin_lat', default_value='63.4391',
            description='ENU origin latitude'),
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
                'use_tcp_tunnel':  use_tcp_tunnel,
                'tcp_listen_port': tcp_listen_port,
                'adb_host':        adb_host,
                'adb_port':        adb_port,
            }],
        ),

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
        Node(
            package='car_control',
            executable='dashboard_server.py',
            name='dashboard_server',
            output='screen',
        ),
    ])

