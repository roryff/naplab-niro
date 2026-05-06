import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('car_control').perform(context)

    def cfg(name):
        return os.path.join(pkg_share, 'config', name)

    return [
        Node(
            package='car_control',
            executable='comma_node',
            name='comma_node',
            output='screen',
            parameters=[cfg('comma_node.yaml')],
        ),
        Node(
            package='car_control',
            executable='gnss_node',
            name='gnss_node',
            output='screen',
            parameters=[cfg('gnss_node.yaml')],
            arguments=['--ros-args', '--log-level', 'gnss_node:=debug'],
        ),
        Node(
            package='car_control',
            executable='dashboard_server.py',
            name='dashboard_server',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
