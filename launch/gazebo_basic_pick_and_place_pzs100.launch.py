"""PZS100 Gazebo simulation with the basic CBS pick-and-place BT."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
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
            # PZS100 crane bringup -- common PZS arguments are encoded in
            # pzs100_bringup.launch.py (this same package). Only override the
            # things that differ from its defaults.
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                / "launch"
                / "pzs100_bringup.launch.py",
                launch_arguments={
                    "planner": "ilqr",
                    "controller": LaunchConfiguration("controller"),
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
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="concrete_block_behavior_tree",
                        executable="gazebo_block_spawner.py",
                        name="gazebo_block_spawner",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "seed_config_file": PathJoinSubstitution([
                                    FindPackageShare("concrete_block_perception"),
                                    "config",
                                    "world_model_seed_pick_place.yaml",
                                ]),
                                "gazebo_world_frame": "world",
                                "use_precomputed_gazebo_pose": False,
                                "seed_frame_id": "K0_mounting_base",
                                # gazebo_base.launch.py spawns the crane entity at
                                # y=-6, yaw=pi.  Combined with the URDF fixed
                                # joints, K0_mounting_base lands here in Gazebo.
                                "gazebo_seed_frame_xyz": [6.93852, -6.35, 1.1407],
                                "gazebo_seed_frame_rpy_deg": [0.0, 0.0, 0.0],
                                "spawn_height_offset": 0.15,
                                "sync_world_model_from_gazebo": False,
                                "settle_time_sec": 3.0,
                                "service_wait_timeout_sec": 60.0,
                                "gazebo_get_entity_state_service": "/gazebo/get_entity_state",
                            },
                        ],
                    ),
                ],
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
