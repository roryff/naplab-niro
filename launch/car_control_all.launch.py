import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def gstreamer_pipeline(multicast_ip, port=10030):
    """Build GStreamer pipeline for UDP multicast HEVC stream."""
    return (
        f"udpsrc uri=udp://{multicast_ip}:{port} buffer-size=4194304 ! "
        "rtph265depay ! h265parse ! avdec_hevc ! videoconvert ! appsink"
    )


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    pkg_share = FindPackageShare('car_control').perform(context)

    control_mode  = LaunchConfiguration('control_mode').perform(context)
    path_csv_file = LaunchConfiguration('path_csv_file').perform(context)
    desired_speed = LaunchConfiguration('desired_speed_mps').perform(context)
    enable_cameras = LaunchConfiguration('enable_cameras').perform(context)

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
                    'use_gst_timestamps': True,
                    'sync': False,
                    'drop': True,
                }],
                remappings=[
                    ('image_raw', topic),
                ],
            ))

    # ---- dashboard_server (always) ---------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='dashboard_server.py',
        name='dashboard_server',
        output='screen',
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
        DeclareLaunchArgument('desired_speed_mps', default_value='4.0',
            description='Desired driving speed [m/s]'),

        # ---- Cameras ---------------------------------------------------------------
        DeclareLaunchArgument('enable_cameras', default_value='false',
            description='Enable multi-camera UDP multicast nodes (true/false)'),

        OpaqueFunction(function=launch_setup),
    ])
