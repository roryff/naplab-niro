"""
Main launch file for the autonomous driving stack.

Always starts: comma_node, gnss_node, robot_state_publisher, dashboard_server, foxglove_bridge.

Selectable control architecture via control_mode:
  unified  (default) — lateral_mpc_node: single 4-state MPC [CTE, dPsi, delta, dRate]
  cascade            — path_follower_node (Stanley) + steering_mpc_node (torque MPC)
 
Optional sensors:
  enable_cameras:=true   — 4× UDP multicast H.265→H.264 camera nodes
  enable_lidar:=true     — 2× Ouster OS1 lidar nodes (lifecycle-managed)

Examples:
    ros2 launch car_control drive.launch.py
    ros2 launch car_control drive.launch.py desired_speed_mps:=8.0 enable_cameras:=true
    ros2 launch car_control drive.launch.py control_mode:=cascade
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    pkg_share = FindPackageShare('car_control').perform(context)

    control_mode   = LaunchConfiguration('control_mode').perform(context)
    path_csv_file  = LaunchConfiguration('path_csv_file').perform(context)
    desired_speed  = LaunchConfiguration('desired_speed_mps').perform(context)
    enable_cameras = LaunchConfiguration('enable_cameras').perform(context)
    enable_lidar   = LaunchConfiguration('enable_lidar').perform(context)

    if not path_csv_file:
        path_csv_file = os.path.join(pkg_share, 'paths', 'path.csv')

    def cfg(name):
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
        # path_follower_node (Stanley) → steering_mpc_node (torque MPC)
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
                {'desired_speed_mps': float(desired_speed)},
            ],
        ))

    elif control_mode == 'unified':
        # lateral_mpc_node: single MPC replacing path_follower + steering_mpc
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

    # ---- cameras (conditional) ------------------------------------------------
    if enable_cameras.lower() == 'true':
        import importlib.util as _ilu
        _spec = _ilu.spec_from_file_location(
            'launch_utils',
            os.path.join(pkg_share, 'launch', 'launch_utils.py'))
        _m = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_m)
        nodes.append(_m.build_camera_container(pkg_share))

    # ---- dashboard + foxglove (always) ----------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='dashboard_server.py',
        name='dashboard_server',
        output='screen',
    ))
    nodes.append(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8766}],
    ))

    # ---- lidar nodes (conditional) --------------------------------------------
    if enable_lidar.lower() == 'true':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'lidars.launch.py')
            )
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode', default_value='unified',
            description=(
                'Steering control architecture: '
                '"unified" = lateral_mpc_node (default), '
                '"cascade" = path_follower_node + steering_mpc_node'
            )),
        DeclareLaunchArgument(
            'path_csv_file', default_value='',
            description='Path to recorded drive CSV (empty = bundled paths/path.csv)'),
        DeclareLaunchArgument(
            'desired_speed_mps', default_value='5.5',
            description='Desired driving speed [m/s]'),
        DeclareLaunchArgument(
            'enable_cameras', default_value='false',
            description='Enable 4× camera nodes (true/false)'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='false',
            description='Enable Ouster lidar nodes (true/false)'),
        OpaqueFunction(function=launch_setup),
    ])
