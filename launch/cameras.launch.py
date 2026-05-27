"""
Standalone camera launch file.

Starts all four camera nodes in a composable container, plus dashboard_server and
foxglove_bridge for monitoring.  Each camera receives H.265 MPEG-TS over UDP multicast,
hardware-decodes with nvv4l2decoder, re-encodes to H.264 with nvv4l2h264enc (all in
NVMM, zero CPU copy), and publishes foxglove_msgs/msg/CompressedVideo on
cameras/{name}/image_compressed.

Usage:
    ros2 launch car_control cameras.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('car_control').perform(context)

    import importlib.util as _ilu
    _spec = _ilu.spec_from_file_location(
        'launch_utils',
        os.path.join(pkg_share, 'launch', 'launch_utils.py'))
    _m = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_m)

    return [
        _m.build_camera_container(pkg_share),
        Node(
            package='car_control',
            executable='dashboard_server.py',
            name='dashboard_server',
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
