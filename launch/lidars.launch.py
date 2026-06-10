"""Launch two Ouster lidar drivers with lifecycle state management."""

from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import (RegisterEventHandler, EmitEvent, LogInfo)
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action


def make_lidar_nodes(ns: str, params_file: str):
    """Return the LifecycleNode + lifecycle event actions for one lidar."""

    node = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace=ns,
        parameters=[params_file],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'[{ns}] activating...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True,
        )
    )

    finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='finalized',
            entities=[
                LogInfo(msg=f'[{ns}] sensor entered finalized state (error or shutdown).'),
            ],
        )
    )

    return [node, configure_event, activate_event, finalized_event]


def generate_launch_description():
    car_control_pkg_dir = get_package_share_directory('car_control')
    lidars_yaml = str(Path(car_control_pkg_dir) / 'config' / 'lidars.yaml')

    sensors = ['lidar_side', 'lidar_top']  # third sensor (192.168.2.228) is in ERROR state

    actions = []
    for ns in sensors:
        actions.extend(make_lidar_nodes(ns, lidars_yaml))

    return launch.LaunchDescription(actions)
