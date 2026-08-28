"""
PZS100 Gazebo simulation with the CBS wall assembly BT, new control stack.

The CBS-only twin of `gazebo_wall_assembly_pzs100.launch.py`.  Everything above
the crane is unchanged -- the same world model, wall plan server, BT action
server, block spawner, grip trajectory server and `cbs.rviz`.  What changes is
underneath: instead of routing through `epsilon_crane_bringup_sim`'s
`gazebo_model_bt.launch.py` (which pulls in `epsilon_crane_bringup_mp`'s
`a2b_ilqr_server` unconditionally), this includes `crane_bringup/sim.launch.py`
and gets the new architecture -- `crane_planner`, `crane_mpc`,
`crane_supervisor`, and `crane_velocity_controller` with
`trajectory_controller_a2b` chained onto it.

The seam is the `a2b_movement` service.  `crane_planning` serves the same
`timber_crane_planning_interfaces/srv/CalcMovement` the BT's `CalcA2BMovement`
node calls, so the tree needs no change to reach the new planner.

`crane_mpc` is started but is not in the BT's execution path: the tree sends its
trajectory to `/trajectory_controller_a2b/follow_joint_trajectory`, which is
chained onto `crane_velocity_controller`. The MPC consumes `/crane/reference`
from the planner and ships in `shadow` mode until the supervisor is asked for
`mpc` through `/crane/set_mode`. There is therefore no `controller:=pid|mpc`
argument here; the choice is made at runtime, not at launch.

Two things do not line up on their own and are handled here:

* `sim.launch.py` publishes the deadman on `/crane/remote_ctrl_states`, while
  the BT's `CheckUserApproval` / `GetUserApproval` listen on
  `/gpio_controller/remote_ctrl_states`.  A second `sim_remote` is started with
  a remap rather than editing either side.
* `crane_bringup/sim.launch.py` starts no RViz, so it is started here with
  `cbs.rviz` -- which already carries an enabled display for the planner's
  `/crane_planner/planned_path`.

Three things are known to be unfinished, and are left visible rather than
papered over:

* **No motion will plan yet.**  `CalcMovement.srv` defaults
  `check_gripper_collision` to true and the BT's `CalcA2BMovement` never
  overrides it, so `a2b_adapter.cpp` sets `avoid_collisions` and
  `planner_core.cpp` refuses -- nothing in this workspace publishes
  `/crane/collision_scene`.  Publishing an empty scene would clear the refusal,
  but that refusal is deliberate: it exists so a trajectory never reads as
  checked when no scene arrived.  A real producer is the fix.
* **Two owners of `trajectory_controller_a2b`.**  `sim.launch.py` spawns it
  active, `subtree_execute_trajectory.xml` activates and deactivates it around
  every motion, and `crane_supervisor` treats itself as the only caller of
  `/controller_manager/switch_controller` and polls `list_controllers` STRICT.
  The timber chain avoided this by spawning the controller `--inactive`.
* **Gripper TF is uncorrected.**  `pzs100_rviz_joint_state_adapter.py` inverted
  the EPSCOPE actuator read on `q9_left_rail_joint` and synthesised
  `q11_right_rail_joint`; it fed a second `robot_state_publisher` that owned
  `/tf` while the raw one was remapped to `tf_gazebo`.  `sim.launch.py` runs a
  single publisher on raw `/joint_states`, so re-adding the adapter here would
  need that two-publisher split rather than one more node.

`start_perception` is carried over for parity but is currently inert:
`sim.launch.py` hard-codes `enable_livox_sim:=''`, so no simulated sensor
publishes the cloud the detector subscribes to.
"""

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
    PathSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Plugin config ────────────────────────────────────────────────────
    # Identical to the timber-backed twin: epsilon_crane's default.yaml is the
    # base, CBS's bt_server_override.yaml widens plugin_lib_names.  The BT
    # plugins themselves are stack-agnostic -- they speak service and action
    # names, and those names are the same on both stacks.
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
    grasp_detector_config = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "gripper_grasp_detector_sim.yaml"
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
                "initial_pose",
                default_value="1",
                description="Crane initial pose preset passed to crane_bringup/sim.",
            ),
            DeclareLaunchArgument(
                "gazebo_world_file",
                default_value="epsilon_crane.world",
                description=(
                    "Gazebo world, resolved inside testsite_description/worlds. "
                    "Named as in the timber twin; sim.launch.py calls it `world`."
                ),
            ),
            DeclareLaunchArgument(
                "place_approach_angle_deg",
                default_value="4.0",
            ),
            DeclareLaunchArgument("lift_height", default_value="1.0"),
            DeclareLaunchArgument("start_perception", default_value="false"),
            DeclareLaunchArgument("start_grasp_detector", default_value="True"),
            DeclareLaunchArgument(
                "start_tui",
                default_value="false",
                description=(
                    "Start timber_crane_tui's keyboard remote. Off by default: the "
                    "package lives under src/legacy and is not built here."
                ),
            ),
            DeclareLaunchArgument("lidar_points_topic", default_value="/livox/points"),
            DeclareLaunchArgument(
                "enable_placement_disturbance", default_value="false"
            ),
            DeclareLaunchArgument("placement_disturbance_x_m", default_value="0.06"),
            DeclareLaunchArgument("placement_disturbance_y_m", default_value="-0.04"),
            DeclareLaunchArgument("placement_disturbance_z_m", default_value="0.00"),
            DeclareLaunchArgument(
                "placement_disturbance_yaw_rad", default_value="0.035"
            ),
            DeclareLaunchArgument(
                "seed_file",
                default_value=PathSubstitution(
                    FindPackageShare("concrete_block_world_model")
                )
                / "config"
                / "world_model_seed_pick_place.yaml",
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_PACKAGE",
                value="concrete_block_behavior_tree",
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_CATALOG",
                value=PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                / "config"
                / "bt_panel_catalog.yaml",
            ),
            # The panel's fixed "Move empty" button, which is hard-wired to one
            # tree.  Its default is `move_empty.xml`, the epsilon name; this
            # package has no such file, so without this the button would stay
            # disabled and the CBS move-empty tree would only be reachable
            # through the catalog drop-down.
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_MOVE_EMPTY",
                value="/behavior_trees/cbs_move_empty_pzs100.xml",
            ),
            # ── The new architecture, whole ──────────────────────────────
            # sim.launch.py brings Gazebo, the description (tool is wired to
            # pzs100_description inside it), the spawn, joint_state_broadcaster,
            # crane_velocity_controller + trajectory_controller_a2b,
            # pendulum_state_broadcaster, crane_supervisor, crane_planner and
            # crane_mpc.  Nothing from the timber motion-planning tree.
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("crane_bringup"))
                / "launch"
                / "sim.launch.py",
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "world": LaunchConfiguration("gazebo_world_file"),
                }.items(),
            ),
            # ── Deadman bridge ──────────────────────────────────────────
            # sim.launch.py's own sim_remote publishes /crane/remote_ctrl_states
            # for crane_supervisor.  The BT reads the epsilon name instead, so a
            # second publisher is remapped onto it.  Two independent publishers
            # on two topics -- no relay, no edit to either stack.
            Node(
                package="crane_bringup",
                executable="sim_remote",
                name="bt_operator_remote",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    (
                        "/crane/remote_ctrl_states",
                        "/gpio_controller/remote_ctrl_states",
                    )
                ],
                output="log",
            ),
            # ── RViz ────────────────────────────────────────────────────
            # cbs.rviz already has an enabled Path display on
            # /crane_planner/planned_path, which only this stack fills.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                    / "rviz"
                    / "cbs.rviz",
                ],
                parameters=[{"use_sim_time": True}],
                # cbs.rviz's RobotModel display subscribes to /robot_description,
                # while sim.launch.py publishes the description on
                # /robot_description_full (the name the gazebo_ros2_control
                # plugin hard-codes). Without this the robot never appears.
                remappings=[("/robot_description", "/robot_description_full")],
                condition=IfCondition(LaunchConfiguration("gui")),
                output="log",
            ),
            # ── Perception (off by default, as in the timber twin) ───────
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="livox_sim_sensor_frame",
                arguments=["0", "0", "0", "0", "0", "0", "livox_frame", "livox_sim"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="concrete_block_detector",
                executable="concrete_block_detector_node",
                name="concrete_block_detector",
                output="screen",
                parameters=[
                    PathSubstitution(FindPackageShare("concrete_block_detector"))
                    / "config"
                    / "detector.yaml",
                    {
                        "use_sim_time": True,
                        "point_cloud_transport": "raw",
                    },
                ],
                remappings=[("points", LaunchConfiguration("lidar_points_topic"))],
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            # ── World model ─────────────────────────────────────────────
            # Same seed YAML the Gazebo block spawner reads, so markers,
            # GetCoarseBlocks and the spawned blocks share one source.
            Node(
                package="concrete_block_world_model",
                executable="world_model_node",
                name="world_model_node",
                parameters=[
                    PathSubstitution(FindPackageShare("concrete_block_world_model"))
                    / "config"
                    / "world_model.yaml",
                    seed_file,
                    {
                        "use_sim_time": True,
                        "world_frame": "world",
                        "refine_grasped.tcp_frame": "K8_tool_center_point",
                        "pipeline_mode": "idle",
                        "perception_mode": "IDLE",
                    },
                ],
                remappings=[
                    ("block_world_model", "/cbp/block_world_model"),
                    ("block_world_model_markers", "/cbp/block_world_model_markers"),
                    ("block_goal_markers", "/cbp/block_goal_markers"),
                ],
                output="screen",
            ),
            # ── Grasp detector ──────────────────────────────────────────
            Node(
                package="concrete_block_behavior_tree",
                executable="gripper_grasp_detector",
                name="gripper_grasp_detector",
                output="screen",
                parameters=[grasp_detector_config, {"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_grasp_detector")),
            ),
            # ── Grip trajectory server ──────────────────────────────────
            # The CBS simple server, not timber's.  It also publishes the
            # virtual TCP static TF from its tcp_z_offset parameter.
            Node(
                package="concrete_block_motion_planning",
                executable="grip_traj_server_simple.py",
                name="grip_traj_server",
                output="screen",
                parameters=[
                    PathSubstitution(FindPackageShare("concrete_block_motion_planning"))
                    / "config"
                    / "grip_traj_simple.yaml",
                    {
                        "lift_height": ParameterValue(
                            LaunchConfiguration("lift_height"),
                            value_type=float,
                        ),
                    },
                    {"use_sim_time": True},
                ],
            ),
            # ── Wall plan server ────────────────────────────────────────
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
                        "place_approach_angle_deg": ParameterValue(
                            LaunchConfiguration("place_approach_angle_deg"),
                            value_type=float,
                        ),
                        "wall_plans_file": PathSubstitution(
                            FindPackageShare("concrete_block_assembly_planning")
                        )
                        / "config"
                        / "wall_plans.yaml",
                    },
                ],
            ),
            # ── BT action server ────────────────────────────────────────
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": True},
                    {
                        "simulated_placement_disturbance.enabled": ParameterValue(
                            LaunchConfiguration("enable_placement_disturbance"),
                            value_type=bool,
                        ),
                        "simulated_placement_disturbance.x_m": ParameterValue(
                            LaunchConfiguration("placement_disturbance_x_m"),
                            value_type=float,
                        ),
                        "simulated_placement_disturbance.y_m": ParameterValue(
                            LaunchConfiguration("placement_disturbance_y_m"),
                            value_type=float,
                        ),
                        "simulated_placement_disturbance.z_m": ParameterValue(
                            LaunchConfiguration("placement_disturbance_z_m"),
                            value_type=float,
                        ),
                        "simulated_placement_disturbance.yaw_rad": ParameterValue(
                            LaunchConfiguration("placement_disturbance_yaw_rad"),
                            value_type=float,
                        ),
                    },
                    {
                        "behaviortree": PathSubstitution(
                            FindPackageShare("lsrl_behavior_tree")
                        )
                        / "behavior_trees"
                        / "empty.xml"
                    },
                ],
            ),
            # ── Lifecycle manager ───────────────────────────────────────
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": True},
                ],
            ),
            # ── Gazebo block spawner ────────────────────────────────────
            # The 8 s delay and the world offset are carried over verbatim:
            # sim.launch.py spawns the crane at y=-6, yaw=pi, exactly as
            # gazebo_base.launch.py did, so the seed transform still holds.
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
                                "gazebo_world_frame": "world",
                                "use_precomputed_gazebo_pose": False,
                                "seed_frame_id": "world",
                                "gazebo_seed_frame_xyz": [0.0, -6.0, 0.0],
                                "gazebo_seed_frame_rpy_deg": [0.0, 0.0, 180.0],
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
            # ── Keyboard TUI ────────────────────────────────────────────
            # timber_crane_tui lives under src/legacy and is not built here, so
            # unlike the timber twin this is off unless asked for.
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
                            LaunchConfiguration("start_tui"),
                            "'.lower() in ('true', '1', 'yes')",
                        ]
                    )
                ),
            ),
        ]
    )
