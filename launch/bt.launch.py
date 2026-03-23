import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "default.yaml"]
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
            DeclareLaunchArgument("bt_params_file", default_value=default_cfg),
            DeclareLaunchArgument("keyboard_node", default_value="False"),
            DeclareLaunchArgument("gui", default_value="True"),
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    LaunchConfiguration("bt_params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[
                    LaunchConfiguration("bt_params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
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
