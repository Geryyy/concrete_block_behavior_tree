"""PZS100 Gazebo simulation with the basic CBS pick-and-place BT."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PathSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_bt_config = (
        PathSubstitution(FindPackageShare("epsilon_crane_behavior_tree"))
        / "config"
        / "default.yaml"
    )
    override_bt_config = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "bt_server_override.yaml"
    )
    basic_bt_profile = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "profiles"
        / "basic_pick_and_place.yaml"
    )

    if os.path.exists("/usr/bin/xterm"):
      spawn_terminal_prefix = "xterm -e "
    elif os.path.exists("/usr/bin/gnome-terminal"):
      spawn_terminal_prefix = "gnome-terminal -- "
    else:
      spawn_terminal_prefix = ""

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("controller", default_value="pid"),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_PACKAGE",
                value="concrete_block_behavior_tree",
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_CATALOG",
                value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_behavior_tree"),
                        "config",
                        "bt_panel_catalog.yaml",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_sim"))
                / "launch"
                / "gazebo_model_bt.launch.py",
                launch_arguments={
                    "tool": "pzs100_description",
                    "controller_common_config": "crane_controller_hydraulic_common_pzs100.ros2_control.yaml",
                    "controller_a2b_config_package": PythonExpression(
                        [
                            "'epsilon_crane_bringup_sim' if '",
                            LaunchConfiguration("controller"),
                            "' == 'pid' else 'timber_crane_unconstrained_mpc_controller_cpp'",
                        ]
                    ),
                    "controller_a2b_config": PythonExpression(
                        [
                            "'ros2_control/crane_controller_hydraulic_a2b_jtc_pid_pzs100.ros2_control.yaml' if '",
                            LaunchConfiguration("controller"),
                            "' == 'pid' else 'crane_controller_hydraulic_a2b_jtc_mpc_pzs100.ros2_control.yaml'",
                        ]
                    ),
                    "mp_launch_file": "mp.launch.py",
                    "mp_param_file": "mp_parameter_pzs100.yaml",
                    "grip_traj_param_file": "grip_traj_parameter_pzs100.yaml",
                    "estimators_param_file": "estimators_pzs100.yaml",
                    "enable_gripper_tip_publisher": "False",
                    "start_bt_action_server": "False",
                    "start_grip_traj_server": "False",
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "gui": LaunchConfiguration("gui"),
                }.items(),
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="grip_traj_server_simple.py",
                name="grip_traj_server",
                output="screen",
                parameters=[
                    PathSubstitution(FindPackageShare("concrete_block_motion_planning"))
                    / "config"
                    / "grip_traj_simple.yaml",
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    basic_bt_profile,
                    {"use_sim_time": True},
                    {
                        "behaviortree": PathSubstitution(
                            FindPackageShare("concrete_block_behavior_tree")
                        )
                        / "behavior_trees"
                        / "basic_pick_and_place.xml"
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[base_bt_config, {"use_sim_time": True}],
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
                            "'",
                            LaunchConfiguration("gui"),
                            "'.lower() in ('true', '1', 'yes')",
                        ]
                    )
                ),
            ),
        ]
    )
