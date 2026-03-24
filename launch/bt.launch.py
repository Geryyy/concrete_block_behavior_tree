import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _is_set(value: str) -> bool:
    return bool(value) and value.lower() not in {"none", "false"}


def _bt_runtime_nodes(context):
    params_files = []
    legacy_params = LaunchConfiguration("bt_params_file").perform(context).strip()

    if _is_set(legacy_params):
        params_files.append(legacy_params)
    else:
        for key in (
            "bt_common_params_file",
            "bt_mode_params_file",
            "bt_profile_params_file",
        ):
            value = LaunchConfiguration(key).perform(context).strip()
            if _is_set(value):
                params_files.append(value)

    shared_parameters = params_files + [
        {"use_sim_time": LaunchConfiguration("use_sim_time")}
    ]

    return [
        Node(
            package="lsrl_behavior_tree",
            executable="bt_action_server",
            output="both",
            parameters=shared_parameters,
            condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            output="screen",
            parameters=shared_parameters,
            condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
        ),
    ]


def generate_launch_description():
    common_cfg = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "bt_common.yaml"]
    )
    assembly_cfg = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "bt_assembly.yaml"]
    )
    assembly_profile_cfg = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "profiles",
            "assembly.yaml",
        ]
    )
    if os.path.exists("/usr/bin/xterm"):
        spawn_terminal_prefix = "xterm -e "
    elif os.path.exists("/usr/bin/gnome-terminal"):
        spawn_terminal_prefix = "gnome-terminal -- "
    else:
        raise RuntimeError(
            "could not find any terminal gui, tested for gnome-terminal and xterm"
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_bt_action_server", default_value="true"),
            DeclareLaunchArgument(
                "bt_params_file",
                default_value="",
                description="Legacy one-file BT config override. Prefer bt_common/mode/profile params.",
            ),
            DeclareLaunchArgument(
                "bt_common_params_file",
                default_value=common_cfg,
            ),
            DeclareLaunchArgument(
                "bt_mode_params_file",
                default_value=assembly_cfg,
            ),
            DeclareLaunchArgument(
                "bt_profile_params_file",
                default_value=assembly_profile_cfg,
            ),
            DeclareLaunchArgument("keyboard_node", default_value="False"),
            DeclareLaunchArgument("gui", default_value="True"),
            OpaqueFunction(function=_bt_runtime_nodes),
            Node(
                package="timber_crane_tui",
                executable="remote_ctrl",
                output="screen",
                parameters=[{"sample_time_ms": 100}],
                prefix=spawn_terminal_prefix,
                condition=IfCondition(
                    PythonExpression(
                        [
                            LaunchConfiguration("keyboard_node"),
                            " and ",
                            LaunchConfiguration("gui"),
                        ]
                    )
                ),
            ),
        ]
    )
