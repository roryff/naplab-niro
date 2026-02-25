from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="car_control",
                executable="gnss_node",
                name="gnss_node",
                output="screen",
            ),
            Node(
                package="car_control",
                executable="comma_node",
                name="comma_node",
                output="screen",
            ),
            Node(
                package="car_control",
                executable="path_follower_node",
                name="path_follower_node",
                output="screen",
            ),
            Node(
                package="car_control",
                executable="dashboard_server.py",
                name="dashboard_server",
                output="screen",
            ),
        ]
    )
