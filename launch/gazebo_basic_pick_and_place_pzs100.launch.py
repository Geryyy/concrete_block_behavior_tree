"""PZS100 Gazebo simulation with the CBS single-block pick-and-place BT."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PathSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    seed_file = LaunchConfiguration("seed_file")

    if os.path.exists("/usr/bin/xterm"):
        spawn_terminal_prefix = "xterm -e "
    elif os.path.exists("/usr/bin/gnome-terminal"):
        spawn_terminal_prefix = "gnome-terminal -- "
    else:
        spawn_terminal_prefix = ""

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument(
                "gazebo_world_file", default_value="epsilon_crane.world"
            ),
            # 2 (`initialization_outside.yaml`), not 1: pose 1 holds the arm at
            # 1.5708 rad, 0.246 above the control-safe arm row (1.324864), so
            # the first move is refused -- crane_planner since 231, and
            # `a2b_ilqr_server` on `planner:=ilqr` prints the same row to the
            # digit ("q0[2] is not feasible: -0.91 < 1.71 < 1.32"). Measured,
            # pose 1: the tree fails on its first node and the crane never
            # moves. Pose 2 plans and executes.
            DeclareLaunchArgument(
                "initial_pose",
                default_value="2",
                description="Initial crane pose (see epsilon_crane_description)",
            ),
            # Same CBS pair as the wall launch (217). `basic_pick_and_place.xml`
            # runs cbs_subtree_a2b_movement, whose settle wait needs
            # /crane/sway_settled and crane_mpc is its only publisher: on
            # `controller:=pid` nothing answers and the tree aborts at the
            # second A2B move. The two defaults move together --
            # `reject_cbs_mpc_without_cbs_planner` refuses cbs_mpc* on any other
            # planner.
            DeclareLaunchArgument(
                "controller",
                default_value="cbs_mpc_active",
                choices=["pid", "mpc", "cbs_mpc", "cbs_mpc_active"],
                description=(
                    "Low-level A2B controller: 'cbs_mpc_active' (default) keeps "
                    "the PID JTC and starts the standalone crane_mpc node beside "
                    "it in active mode, so its horizon drives the JTC; 'cbs_mpc' "
                    "composes identically but in shadow mode; 'pid' is the JTC "
                    "with the PID plugin; 'mpc' is the same JTC with the "
                    "in-process JtcMpcPlugin. 'pid' and 'mpc' start no crane_mpc "
                    "at all, so the tree's settle wait never passes. Both "
                    "cbs_mpc values require planner:=cbs, so overriding planner "
                    "alone is a launch-time error"
                ),
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="cbs",
                choices=["ilqr", "vpsto", "cbs"],
                description=(
                    "A2B planner: 'cbs' (default) is crane_planning's "
                    "crane_planner; 'ilqr'/'vpsto' are the legacy timber servers "
                    "-- select one only with controller:=pid or controller:=mpc"
                ),
            ),
            DeclareLaunchArgument(
                "seed_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_world_model"),
                        "config",
                        "world_model_seed_pick_place.yaml",
                    ]
                ),
                description="World-model/Gazebo seed YAML with world_model.initial_blocks.",
            ),
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
            # The panel's "Move empty" button is fixed to one tree per profile.
            # Its default is epsilon's `move_empty.xml`, which this package does
            # not have, so without this the button stays greyed out and the only
            # way to an A2B move is the dropdown -- where the neighbouring CBS
            # entry needs a launch profile this one is not.
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_MOVE_EMPTY",
                value="/behavior_trees/move_empty_pzs100.xml",
            ),
            # PZS100 crane bringup -- common PZS arguments are encoded in
            # pzs100_bringup.launch.py (this same package). Only override the
            # things that differ from its defaults.
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                / "launch"
                / "pzs100_bringup.launch.py",
                launch_arguments={
                    "planner": LaunchConfiguration("planner"),
                    "controller": LaunchConfiguration("controller"),
                    "start_bt_action_server": "False",
                    "start_grip_traj_server": "False",
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "gui": LaunchConfiguration("gui"),
                    "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
                    "seed_file": seed_file,
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
                package="concrete_block_assembly_planning",
                executable="wall_plan_server",
                name="concrete_block_wall_plan_server",
                output="screen",
                parameters=[
                    PathSubstitution(
                        FindPackageShare("concrete_block_assembly_planning")
                    )
                    / "config"
                    / "wall_plan_server.yaml",
                    {
                        "use_sim_time": True,
                        "world_model_service": "/world_model_node/get_coarse_blocks",
                        "world_model_timeout_s": 2.0,
                        "output_frame": "world",
                        "wall_plans_file": PathSubstitution(
                            FindPackageShare("concrete_block_assembly_planning")
                        )
                        / "config"
                        / "wall_plans.yaml",
                    },
                ],
            ),
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": True},
                    {
                        # One commander per move (issue 148). `cbs_mpc_active` is
                        # the only value where crane_mpc re-goals the JTC by
                        # topic, so it is the only one where the CBS A2B subtree
                        # drops its FollowJointTrajectory goal and waits on
                        # WaitForMoveComplete instead. Left false under the
                        # default controller, two commanders fight over
                        # ~/joint_trajectory and the goal's tolerances.
                        "a2b_mpc_drives": ParameterValue(
                            PythonExpression(
                                [
                                    "'",
                                    LaunchConfiguration("controller"),
                                    "' == 'cbs_mpc_active'",
                                ]
                            ),
                            value_type=bool,
                        ),
                    },
                    {
                        "behaviortree": PathSubstitution(
                            FindPackageShare("concrete_block_behavior_tree")
                        )
                        / "behavior_trees"
                        / "basic_pick_and_place.xml"
                    },
                ],
            ),
            # `node_names`/`autostart` live in bt_server_override.yaml -- upstream
            # dropped the lifecycle_manager block from epsilon's default.yaml, so
            # base_bt_config alone aborts the manager on an uninitialized
            # `node_names`, bt_action_server never activates and `execute_bt`
            # never appears. Same parameter pair as the wall-assembly launch, and
            # the second file is also what gives the BT server a
            # `wait_for_service_timeout` long enough to survive autostart.
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[base_bt_config, override_bt_config, {"use_sim_time": True}],
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
                                "seed_config_file": seed_file,
                                "use_precomputed_gazebo_pose": False,
                                # Seed is in `world` (same frame as wall_spec / plan).
                                "seed_frame_id": "world",
                                # Pose of the ROS `world` frame in the Gazebo world
                                # frame. gazebo_base.launch.py spawns the crane at
                                # y=-6, yaw=pi, so the two worlds differ by yaw 180 +
                                # a -6 m y shift.
                                "gazebo_seed_frame_xyz": [0.0, -6.0, 0.0],
                                "gazebo_seed_frame_rpy_deg": [0.0, 0.0, 180.0],
                                "spawn_height_offset": 0.15,
                            },
                        ],
                    ),
                    Node(
                        package="concrete_block_behavior_tree",
                        executable="gazebo_grasp_weld.py",
                        name="gazebo_grasp_weld",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
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
