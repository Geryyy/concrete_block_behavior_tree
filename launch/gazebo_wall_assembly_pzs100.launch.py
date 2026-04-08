"""PZS100 Gazebo simulation with CBS wall assembly behavior tree.

Launches the full PZS100 crane simulation (without the epsilon_crane BT)
and adds the CBS wall assembly pipeline: wall_plan_server, world_model,
and a bt_action_server with CBS + epsilon_crane plugins.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PathSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = "true"

    # ── Plugin config ────────────────────────────────────────────────────
    # epsilon_crane default.yaml provides base settings (loop duration, lifecycle
    # manager, logging). bt_server_override.yaml overrides plugin_lib_names to
    # include both epsilon_crane and CBS plugins.
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

    # Keyboard TUI
    if os.path.exists("/usr/bin/xterm"):
        spawn_terminal_prefix = "xterm -e "
    elif os.path.exists("/usr/bin/gnome-terminal"):
        spawn_terminal_prefix = "gnome-terminal -- "
    else:
        spawn_terminal_prefix = ""

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="True"),
            # Point the RViz BT panel to concrete_block_behavior_tree
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
            # ── PZS100 crane simulation (without epsilon_crane BT) ───────
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_sim"))
                / "launch"
                / "gazebo_model_bt_pzs100.launch.py",
                launch_arguments={
                    "start_bt_action_server": "False",
                }.items(),
            ),
            # ── World model ──────────────────────────────────────────────
            Node(
                package="concrete_block_perception",
                executable="world_model_node",
                name="world_model_node",
                parameters=[
                    PathSubstitution(FindPackageShare("concrete_block_perception"))
                    / "config"
                    / "world_model.yaml",
                    PathSubstitution(FindPackageShare("concrete_block_perception"))
                    / "config"
                    / "world_model_seed_pick_place.yaml",
                    {
                        "use_sim_time": True,
                        "pipeline_mode": "idle",
                        "perception_mode": "IDLE",
                    },
                ],
                remappings=[
                    ("block_world_model", "/cbp/block_world_model"),
                    ("block_world_model_markers", "/cbp/block_world_model_markers"),
                ],
                output="screen",
            ),
            # ── Wall plan server (lightweight) ───────────────────────────
            Node(
                package="concrete_block_motion_planning",
                executable="wall_plan_server.py",
                name="concrete_block_motion_planning_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            # ── BT action server ─────────────────────────────────────────
            # Loads base config from epsilon_crane, then overrides plugins
            # to include CBS plugins. Same lifecycle pattern as timber framework.
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": True},
                    {
                        "behaviortree": PathSubstitution(
                            FindPackageShare("lsrl_behavior_tree")
                        )
                        / "behavior_trees"
                        / "empty.xml"
                    },
                ],
            ),
            # ── Lifecycle manager ────────────────────────────────────────
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[
                    base_bt_config,
                    {"use_sim_time": True},
                ],
            ),
            # ── Keyboard TUI ────────────────────────────────────────────
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
