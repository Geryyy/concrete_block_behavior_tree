from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "default.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_bt_action_server", default_value="true"),
            DeclareLaunchArgument("bt_params_file", default_value=default_cfg),
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="screen",
                parameters=[
                    LaunchConfiguration("bt_params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("bt_params_file"),
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        ],
                        condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
                    )
                ],
            ),
        ]
    )
