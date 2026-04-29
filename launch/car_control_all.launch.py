import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    pkg_share = FindPackageShare('car_control').perform(context)

    control_mode  = LaunchConfiguration('control_mode').perform(context)
    path_csv_file = LaunchConfiguration('path_csv_file').perform(context)
    desired_speed = LaunchConfiguration('desired_speed_mps').perform(context)

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

        OpaqueFunction(function=launch_setup),
    ])
