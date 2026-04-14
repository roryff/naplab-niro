from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('car_control').perform(context)

    control_mode  = LaunchConfiguration('control_mode').perform(context)
    path_csv_file = LaunchConfiguration('path_csv_file').perform(context)
    desired_speed = LaunchConfiguration('desired_speed_mps').perform(context)
    model_config  = LaunchConfiguration('model_config_path').perform(context)
    origin_lat    = LaunchConfiguration('origin_lat').perform(context)
    origin_lon    = LaunchConfiguration('origin_lon').perform(context)
    origin_alt    = LaunchConfiguration('origin_alt').perform(context)
    use_tcp       = LaunchConfiguration('use_tcp_tunnel').perform(context)
    tcp_port      = LaunchConfiguration('tcp_listen_port').perform(context)
    adb_host      = LaunchConfiguration('adb_host').perform(context)
    adb_port      = LaunchConfiguration('adb_port').perform(context)
    weight_cte    = LaunchConfiguration('weight_cte').perform(context)
    weight_psi    = LaunchConfiguration('weight_psi').perform(context)
    weight_torque = LaunchConfiguration('weight_torque').perform(context)

    # Default path to bundled path.csv when none supplied
    if not path_csv_file:
        import os
        path_csv_file = os.path.join(pkg_share, 'paths', 'path.csv')

    nodes = []

    # ---- comma_node (always) ---------------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='comma_node',
        name='comma_node',
        output='screen',
        parameters=[{
            'use_tcp_tunnel':  use_tcp == 'true',
            'tcp_listen_port': int(tcp_port),
            'adb_host':        adb_host,
            'adb_port':        int(adb_port),
        }],
    ))

    # ---- gnss_node (always) ----------------------------------------------------
    nodes.append(Node(
        package='car_control',
        executable='gnss_node',
        name='gnss_node',
        output='screen',
        parameters=[{
            'auto_set_origin': False,
            'origin_lat': float(origin_lat),
            'origin_lon': float(origin_lon),
            'origin_alt': float(origin_alt),
        }],
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
            parameters=[{
                'path_csv_file':     path_csv_file,
                'desired_speed_mps': float(desired_speed),
                'auto_enable':       True,
            }],
            remappings=[
                ('cmd_vel', 'path_follower/cmd_vel'),
            ],
        ))
        nodes.append(Node(
            package='car_control',
            executable='steering_mpc_node',
            name='steering_mpc_node',
            output='screen',
            parameters=[{
                'model_config_path': model_config,
                'desired_speed_mps': float(desired_speed),
                'dt':      0.05,
                'horizon': 40,
                'tau_r':   0.78,
                'gain_r':  36.0,
            }],
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
            parameters=[{
                'path_csv_file':     path_csv_file,
                'desired_speed_mps': float(desired_speed),
                'auto_enable':       False,
                'horizon':           40,
                'tau_r':             0.78,
                'gain_r':            36.0,
                'weight_cte':        float(weight_cte),
                'weight_psi':        float(weight_psi),
                'weight_torque':     float(weight_torque),
            }],
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
    pkg_share = FindPackageShare('car_control')

    return LaunchDescription([
        # ---- Control mode ------------------------------------------------------
        DeclareLaunchArgument(
            'control_mode', default_value='unified',
            description=(
                'Steering control architecture: '
                '"cascade" = path_follower_node + steering_mpc_node (Option A), '
                '"unified" = lateral_mpc_node only (Option B)'
            )),

        # ---- TCP / ADB connection ----------------------------------------------
        DeclareLaunchArgument('use_tcp_tunnel',  default_value='true',
            description='Listen for comma device over TCP (true) instead of ADB (false)'),
        DeclareLaunchArgument('tcp_listen_port', default_value='3000',
            description='Port to listen on in TCP tunnel mode'),
        DeclareLaunchArgument('adb_host',        default_value='127.0.0.1',
            description='ADB device host (use_tcp_tunnel=false only)'),
        DeclareLaunchArgument('adb_port',        default_value='5555',
            description='ADB device port (use_tcp_tunnel=false only)'),

        # ---- Path / speed ------------------------------------------------------
        DeclareLaunchArgument('path_csv_file', default_value='',
            description='Path to recorded drive CSV (empty = bundled paths/path.csv)'),
        DeclareLaunchArgument('desired_speed_mps', default_value='4.0',
            description='Desired driving speed [m/s]'),
        DeclareLaunchArgument('model_config_path', default_value='',
            description='Path to integrator_model.yaml (cascade mode, empty = use defaults)'),

        # ---- ENU origin --------------------------------------------------------
        DeclareLaunchArgument('origin_lat', default_value='63.4391',
            description='ENU origin latitude  (fixed so all drives share the same frame)'),
        DeclareLaunchArgument('origin_lon', default_value='10.4128',
            description='ENU origin longitude'),
        DeclareLaunchArgument('origin_alt', default_value='3.2',
            description='ENU origin altitude [m]'),

        # ---- Unified MPC weights (unified mode only) ---------------------------
        DeclareLaunchArgument('weight_cte',    default_value='2.0',
            description='[unified] Cost weight on cross-track error'),
        DeclareLaunchArgument('weight_psi',    default_value='1.0',
            description='[unified] Cost weight on heading error'),
        DeclareLaunchArgument('weight_torque', default_value='0.1',
            description='[unified] Cost weight on steering torque (regularisation)'),

        OpaqueFunction(function=launch_setup),
    ])
