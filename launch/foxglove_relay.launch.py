"""
Foxglove relay launch file.

Starts FoxgloveRelayNode instances (H.265 → H.264 preview) plus foxglove_bridge.
Run alongside cameras.launch.py when you want Foxglove Studio preview.

Each relay only holds GPU resources while a Foxglove subscriber is connected.

Usage:
    # Single camera (left only, default)
    ros2 launch car_control foxglove_relay.launch.py

    # All cameras
    ros2 launch car_control foxglove_relay.launch.py cameras:=front,right,rear,left

    # Without foxglove_bridge (if already running)
    ros2 launch car_control foxglove_relay.launch.py with_foxglove_bridge:=false
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('car_control').perform(context)
    cameras = LaunchConfiguration('cameras').perform(context).split(',')

    import importlib.util as _ilu
    _spec = _ilu.spec_from_file_location(
        'launch_utils',
        os.path.join(pkg_share, 'launch', 'launch_utils.py'))
    _m = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_m)

    return [
        _m.build_foxglove_relay_container(pkg_share, cameras=cameras),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_foxglove_bridge')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras',
            default_value='left,right,front,rear',
            description='Comma-separated camera names to relay, e.g. front,left'),
        DeclareLaunchArgument(
            'with_foxglove_bridge',
            default_value='true',
            description='Set false if foxglove_bridge is already running'),
        OpaqueFunction(function=launch_setup),
    ])
