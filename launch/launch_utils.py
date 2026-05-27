"""Shared launch helpers for the car_control package."""

import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_CAMERA_NAMES = ['front', 'right', 'rear', 'left']


def build_camera_container(pkg_share: str):
    """Return a ComposableNodeContainer with all four camera nodes.

    All parameters (shared and per-camera) come from config/cameras.yaml.
    """
    cameras_yaml = os.path.join(pkg_share, 'config', 'cameras.yaml')
    nodes = [
        ComposableNode(
            package='car_control',
            plugin='CameraNode',
            name=f'camera_{name}',
            parameters=[cameras_yaml],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
        for name in _CAMERA_NAMES
    ]

    return ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )
