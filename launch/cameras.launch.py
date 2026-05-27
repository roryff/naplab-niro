"""
Standalone camera launch file.

Starts all four camera nodes in a composable container plus dashboard_server.
Each camera receives H.265 MPEG-TS over UDP multicast and publishes raw
foxglove_msgs/msg/CompressedVideo (format="h265") on cameras/{name}/image_compressed.

For Foxglove Studio preview, also run:
    ros2 launch car_control foxglove_relay.launch.py

That launch provides H.264 preview topics (cameras/{name}/image_compressed_preview)
and foxglove_bridge.  The relay only uses GPU while Foxglove is connected.

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
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
