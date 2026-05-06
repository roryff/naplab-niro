"""
ROS 2 launch file for multi-camera UDP multicast capture.

Launches 4 CameraNode composable components in a single ComposableNodeContainer.
Each CameraNode receives H.265 MPEG-TS over UDP multicast, hardware-decodes with
nvv4l2decoder, hardware-encodes to H.264 with nvv4l2h264enc (all in NVMM, zero
CPU copy), and publishes foxglove_msgs/msg/CompressedVideo on cameras/{name}/image_compressed.

Usage:
    ros2 launch car_control cameras_only.launch.py [enable_cameras:=true]
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    enable_cameras = LaunchConfiguration('enable_cameras').perform(context)
    if enable_cameras.lower() != 'true':
        return []

    cameras = [
        ('front', '239.10.0.1', 'camera_front', 'cameras/front'),
        ('right', '239.10.0.2', 'camera_right', 'cameras/right'),
        ('rear',  '239.10.0.3', 'camera_rear',  'cameras/rear'),
        ('left',  '239.10.0.4', 'camera_left',  'cameras/left'),
    ]

    composable_nodes = []
    for name, multicast_ip, frame_id, ns in cameras:
        compressed_topic = f'{ns}/image_compressed'

        composable_nodes.append(ComposableNode(
            package='car_control',
            plugin='CameraNode',
            name=f'camera_{name}',
            parameters=[{
                'multicast_ip':    multicast_ip,
                'port':            10030,
                'topic':           compressed_topic,
                'frame_id':        frame_id,
                'multicast_iface': 'enP2p1s0',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))

    return [
        ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=composable_nodes,
            output='screen',
        ),
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='true',
            description='Enable camera nodes (true/false)'),
        OpaqueFunction(function=launch_setup),
    ])
