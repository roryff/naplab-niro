"""
ROS 2 launch file for multi-camera UDP multicast capture.

Launches 4 gscam2 instances (front, right, rear, left) capturing HEVC streams
via UDP multicast and publishing CompressedImage on separate topics.

Usage:
    ros2 launch car_control cameras_only.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def gstreamer_pipeline(multicast_ip, port=10030):
    """
    Build GStreamer pipeline for UDP multicast HEVC stream.
    
    Pipeline: udpsrc (multicast) → rtph265depay → h265parse → avdec_hevc → videoconvert → appsink
    
    Args:
        multicast_ip: Multicast group IP (e.g., "239.10.0.1")
        port: UDP port (default 10030)
    
    Returns:
        GStreamer pipeline URI string
    """
    return (
        f"udpsrc uri=udp://{multicast_ip}:{port} buffer-size=4194304 ! "
        "rtph265depay ! h265parse ! avdec_hevc ! videoconvert ! appsink"
    )


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    
    pkg_share = FindPackageShare('car_control').perform(context)
    enable_cameras = LaunchConfiguration('enable_cameras').perform(context)
    
    nodes = []
    
    if enable_cameras.lower() != 'true':
        return nodes
    
    # Camera definitions: (name, multicast_ip, frame_id, topic)
    cameras = [
        ('front', '239.10.0.1', 'camera_front', 'cameras/front/compressed'),
        ('right', '239.10.0.2', 'camera_right', 'cameras/right/compressed'),
        ('rear',  '239.10.0.3', 'camera_rear',  'cameras/rear/compressed'),
        ('left',  '239.10.0.4', 'camera_left',  'cameras/left/compressed'),
    ]
    
    for name, multicast_ip, frame_id, topic in cameras:
        pipeline_uri = gstreamer_pipeline(multicast_ip)
        
        nodes.append(Node(
            package='gscam2',
            executable='gscam2_node',
            name=f'gscam2_{name}',
            output='screen',
            parameters=[{
                'gscam_config': pipeline_uri,
                'frame_id': frame_id,
                'use_gst_timestamps': True,  # Use GStreamer timestamps, not ROS clock
                'sync': False,  # Don't wait for sync on appsink
                'drop': True,  # Drop frames if pipeline can't keep up
            }],
            remappings=[
                ('image_raw', topic),
            ],
        ))
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='false',
            description='Enable camera nodes (true/false)'),
        
        OpaqueFunction(function=launch_setup),
    ])
