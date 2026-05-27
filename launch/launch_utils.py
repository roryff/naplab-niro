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


def build_foxglove_relay_container(pkg_share: str, cameras=None):
    """Return a ComposableNodeContainer with FoxgloveRelayNode instances.

    cameras: list of camera names to relay, e.g. ['left'].
             Defaults to all four if not specified.
    Each relay transcodes one H.265 stream to H.264 preview lazily —
    the GPU pipeline only runs while a Foxglove subscriber is connected.
    """
    if cameras is None:
        cameras = _CAMERA_NAMES
    relay_yaml = os.path.join(pkg_share, 'config', 'foxglove_relay.yaml')
    nodes = [
        ComposableNode(
            package='car_control',
            plugin='FoxgloveRelayNode',
            name=f'relay_{name}',
            parameters=[relay_yaml],
        )
        for name in cameras
    ]

    return ComposableNodeContainer(
        name='foxglove_relay_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )
