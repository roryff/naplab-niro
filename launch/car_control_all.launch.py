import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare



def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    pkg_share = FindPackageShare('car_control').perform(context)

    control_mode  = LaunchConfiguration('control_mode').perform(context)
    path_csv_file = LaunchConfiguration('path_csv_file').perform(context)
    desired_speed = LaunchConfiguration('desired_speed_mps').perform(context)
    enable_cameras = LaunchConfiguration('enable_cameras').perform(context)
    enable_lidar   = LaunchConfiguration('enable_lidar').perform(context)

    # Default path to bundled path.csv when none supplied
    if not path_csv_file:
        path_csv_file = os.path.join(pkg_share, 'paths', 'path.csv')

    def cfg(name):
        """Return the installed path for a node config YAML."""
        return os.path.join(pkg_share, 'config', name)

    nodes = []

    # ---- comma_node (always) ---------------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='comma_node',
        name='comma_node',
        output='screen',
        parameters=[cfg('comma_node.yaml')],
    ))

    # ---- gnss_node (always) ----------------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='gnss_node',
        name='gnss_node',
        output='screen',
        parameters=[cfg('gnss_node.yaml')],
    ))

    # ---- sensor frames / TF tree (always) -------------------------------------
    urdf_path = os.path.join(pkg_share, 'config', 'kia_niro_frames.urdf')
    with open(urdf_path, 'r') as _f:
        robot_description = _f.read()
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    ))

    if control_mode == 'cascade':
        # ---- Option A: path_follower_node + steering_mpc_node ------------------
        #
        # path_follower_node:  Stanley controller → desired angle + 40-step
        #                      reference trajectory on path_follower/steer_ref_traj_deg
        # steering_mpc_node:   Torque MPC using per-step reference trajectory
        #
        nodes.append(Node(
            package='car_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[
                cfg('path_follower_node.yaml'),
                {
                    'path_csv_file':     path_csv_file,
                    'desired_speed_mps': float(desired_speed),
                },
            ],
            remappings=[
                ('cmd_vel', 'path_follower/cmd_vel'),
            ],
        ))
        nodes.append(Node(
            package='car_control',
            executable='steering_mpc_node',
            name='steering_mpc_node',
            output='screen',
            parameters=[
                cfg('steering_mpc_node.yaml'),
                {
                    'desired_speed_mps': float(desired_speed),
                    'model_config_path': cfg('sched_fo2_model.yaml'),
                },
            ],
        ))

    elif control_mode == 'unified':
        # ---- Option B: lateral_mpc_node (replaces both above) -----------------
        #
        # 4-state MPC [CTE, dPsi, delta, dRate] directly minimises cross-track
        # and heading error using path curvature as feed-forward.
        #
        nodes.append(Node(
            package='car_control',
            executable='lateral_mpc_node',
            name='lateral_mpc_node',
            output='screen',
            parameters=[
                cfg('lateral_mpc_node.yaml'),
                {
                    'path_csv_file':     path_csv_file,
                    'desired_speed_mps': float(desired_speed),
                },
            ],
        ))

    else:
        raise RuntimeError(
            f"Unknown control_mode '{control_mode}'. Use 'cascade' or 'unified'.")

    # ---- camera nodes (conditional) -----------------------------------------------
    if enable_cameras.lower() == 'true':
        cameras = [
            ('front', '239.10.0.1', 'camera_front', 'cameras/front'),
            ('right', '239.10.0.2', 'camera_right', 'cameras/right'),
            ('rear',  '239.10.0.3', 'camera_rear',  'cameras/rear'),
            ('left',  '239.10.0.4', 'camera_left',  'cameras/left'),
        ]
        # CameraNode now publishes H.265 bitstream directly as CompressedImage —
        # no hardware decode, no NITROS re-encode, no raw topics on the bus.
        cam_nodes = []
        for name, multicast_ip, frame_id, ns in cameras:
            compressed_topic = f'{ns}/image_compressed'
            cam_nodes.append(ComposableNode(
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
        nodes.append(ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=cam_nodes,
            output='screen',
        ))

    # ---- dashboard_server (always) -----------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='dashboard_server.py',
        name='dashboard_server',
        output='screen',
    ))

    # ---- foxglove_bridge (always) ------------------------------------------------
    nodes.append(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8766}],
    ))

    # ---- lidar nodes (conditional) ---------------------------------------------
    if enable_lidar.lower() == 'true':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'multi_lidar.launch.py')
            )
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ---- Control mode ------------------------------------------------------
        DeclareLaunchArgument(
            'control_mode', default_value='unified',
            description=(
                'Steering control architecture: '
                '"cascade" = path_follower_node + steering_mpc_node (Option A), '
                '"unified" = lateral_mpc_node only (Option B)'
            )),

        # ---- Path / speed ------------------------------------------------------
        DeclareLaunchArgument('path_csv_file', default_value='',
            description='Path to recorded drive CSV (empty = bundled paths/path.csv)'),
        DeclareLaunchArgument('desired_speed_mps', default_value='5.5',
            description='Desired driving speed [m/s]'),

        # ---- Cameras ---------------------------------------------------------------
        DeclareLaunchArgument('enable_cameras', default_value='false',
            description='Enable multi-camera UDP multicast nodes (true/false)'),

        # ---- Lidar -----------------------------------------------------------------
        DeclareLaunchArgument('enable_lidar', default_value='false',
            description='Enable multi-lidar Ouster nodes (true/false)'),

        OpaqueFunction(function=launch_setup),
    ])
