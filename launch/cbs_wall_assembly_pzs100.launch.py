"""
PZS100 Gazebo simulation with the CBS wall assembly BT, new control stack.

The CBS-only twin of `gazebo_wall_assembly_pzs100.launch.py`.  Everything above
the crane is unchanged -- the same world model, wall plan server, BT action
server, block spawner, grip trajectory server and `cbs.rviz`.  What changes is
underneath: instead of routing through `epsilon_crane_bringup_sim`'s
`gazebo_model_bt.launch.py` (which pulls in `epsilon_crane_bringup_mp`'s
`a2b_ilqr_server` unconditionally), this includes `crane_bringup/sim.launch.py`
and gets the new architecture -- `crane_mpc` and `crane_velocity_controller`
with `trajectory_controller_a2b` chained onto it.

The seam is the `a2b_movement` service, and it is the retained
`a2b_ilqr_server` that sits in it here: see `a2b_planner` below. `crane_planner`
serves the same `timber_crane_planning_interfaces/srv/CalcMovement`, so the tree
reaches either without knowing which, but it refuses reachable goals today and
`a2b_planner` therefore defaults to `legacy`.

`crane_supervisor` is **not** started by this profile -- `start_supervisor` is
handed down as `false`. It does not come up at all: it throws in its own
constructor on `tool_controllers: []`, an untyped empty list its shipped config
cannot give a type to. Nothing here needs it -- the tree follows a trajectory on
a controller that is already active and switches nothing -- so what is lost is
`/crane/set_mode`, the mode arbitration and the deadman fault path, none of
which is on this profile's command path.

`initial_pose` defaults to `2` (`initialization_outside.yaml`) and not to the
`1` the timber twin uses. Preset `1` is `initialization_horizontal.yaml`, whose
`theta2_0` is exactly `0.000000` -- the boom's own lower limit. Gazebo sag puts
the measured joint a hair under it and `a2b_ilqr_server` refuses every request
at its start-state check, `q0[1] is not feasible: 0.00 < -0.00 < 1.56`, before
it plans anything. Preset `2` starts the boom at 0.52 rad, well inside.

`crane_mpc` is started but is not in the BT's execution path: the tree sends its
trajectory to `/trajectory_controller_a2b/follow_joint_trajectory`, which is
chained onto `crane_velocity_controller`. The MPC consumes `/crane/reference`,
which nothing publishes while `a2b_planner` is `legacy`, and ships in `shadow`
mode -- and with no supervisor there is nothing to ask for `mpc` anyway. There
is therefore no `controller:=pid|mpc` argument here.

Two things do not line up on their own and are handled here:

* `sim.launch.py` publishes the deadman on `/crane/remote_ctrl_states`, while
  the BT's `CheckUserApproval` / `GetUserApproval` listen on
  `/gpio_controller/remote_ctrl_states`.  A second `sim_remote` is started with
  a remap rather than editing either side -- but only when `start_tui` is
  false.  The TUI publishes the operator's button on that same topic, and its
  message is all-false between keypresses, so running both put a held-true and
  a released-false stream on one topic and every trajectory was cancelled
  ~100 ms in by `GetUserApproval`.  The two are now mutually exclusive: TUI for
  an operated run, `sim_remote` for a headless one.
* `crane_bringup/sim.launch.py` starts no RViz, so it is started here with
  `cbs.rviz` -- which already carries an enabled display for the planner's
  `/crane_planner/planned_path`.

The scene the plan is checked against comes from `world_model_node`, which is
started below: it publishes `/crane/collision_scene` (transient-local, with a
heartbeat) from the same seed YAML the block spawner reads, so the tree's
`CalcA2BMovement` -- which never overrides the `.srv` defaults and therefore
always asks for the collision check -- has a scene to be certified against.

The gripper coordinate is corrected before anything maps it onto the
description.  `gripper_hydraulic.ros2_control.xacro` gives `q9_left_rail_joint`
an EPSCOPE `state_factor` of 2, so `/joint_states` carries the *total* opening
while the URDF joint is one rail; `pzs100_rviz_joint_state_adapter.py` inverts
that onto `/joint_states_rviz`, and `sim.launch.py` is pointed at it through its
`joint_states_topic` argument so the `/tf` publisher, the planner and the MPC
all read one coordinate.  The raw topic is left alone for the grasp detector,
whose `position_window` is written in the EPSCOPE opening.

One thing is known to be unfinished, and is left visible rather than papered
over:

* **Nobody arbitrates `trajectory_controller_a2b`.**  `sim.launch.py` spawns it
  active and `subtree_execute_trajectory.xml` activates and deactivates it
  around every motion.  `crane_supervisor` is meant to be the only caller of
  `/controller_manager/switch_controller` and is not running here, so the tree
  is the only caller and nothing contends with it -- but nothing checks it
  either, and the misreported mode this note used to describe is simply absent
  along with the node that reported it.

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
from launch.conditions import IfCondition, UnlessCondition
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
    # Read by the BT action server only, and by no other launch: it tells the
    # `FollowJointTrajectory` plugin the six joints this stack's chained
    # `trajectory_controller_a2b` declares, so the eight-joint answer both A2B
    # planners give is trimmed instead of rejected.
    cbs_stack_bt_config = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "bt_server_cbs_stack.yaml"
    )
    grasp_detector_config = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "gripper_grasp_detector_sim.yaml"
    )
    seed_file = LaunchConfiguration("seed_file")
    a2b_planner = LaunchConfiguration("a2b_planner")
    use_legacy_a2b = PythonExpression(["'", a2b_planner, "' == 'legacy'"])
    use_native_a2b = PythonExpression(["'", a2b_planner, "' != 'legacy'"])
    mp_param_path = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "motion_planning"
        / "mp_parameter_pzs100.yaml"
    )
    a2b_param_path = (
        PathSubstitution(FindPackageShare("epsilon_crane_bringup_mp"))
        / "config"
        / "motion_planning"
        / "a2b_parameter.yaml"
    )
    collision_objects_path = (
        PathSubstitution(FindPackageShare("epsilon_crane_bringup_mp"))
        / "config"
        / "motion_planning"
        / "collision_objects.yaml"
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
            DeclareLaunchArgument(
                "initial_pose",
                default_value="2",
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
                default_value=LaunchConfiguration("gui"),
                description=(
                    "Start timber_crane_tui's keyboard remote in its own terminal "
                    "(the operator approval prompt). Follows `gui` as in the timber "
                    "twin; set false for a headless run."
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
            # Which node sits in the single `/a2b_movement` seat.
            #
            # `legacy` -- the default -- is `timber_crane_motion_planning`'s
            # `a2b_ilqr_server`, the planner the timber twin has always run,
            # started here with the PZS100 configuration
            # `pzs100_bringup.launch.py` hands it. `crane_planning`'s native
            # planner refuses reachable RViz goals on this profile
            # ("no admissible timing exists for this path:
            # Infeasible_Problem_Detected"), and the tree fails the whole
            # sequence when it does. Everything else about this launch is
            # unchanged: the seam is the service, the BT's `CalcA2BMovement`
            # calls the same name either way, and the trajectory lands on the
            # same `trajectory_controller_a2b` -- the two stacks' controller
            # configs name the same six joints in the same order.
            #
            # `native` restores `crane_planner` and with it `/crane/reference`,
            # the MPC's shadow horizon and `/crane_planner/planned_path`.
            DeclareLaunchArgument(
                "a2b_planner",
                default_value="legacy",
                description=(
                    "Server of /a2b_movement: 'legacy' (a2b_ilqr_server) or "
                    "'native' (crane_planner)."
                ),
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
            # ── The PZS100 gripper coordinate ───────────────────────────
            # `q9_left_rail_joint` reaches /joint_states multiplied by the
            # EPSCOPE `state_factor` of 2 -- the total opening, not the rail the
            # URDF describes. This inverts it onto /joint_states_rviz, which is
            # what every node that maps a joint state onto the description reads
            # below. The raw topic stays as it is: the grasp detector's
            # `position_window` is written in the EPSCOPE opening.
            Node(
                package="concrete_block_behavior_tree",
                executable="pzs100_rviz_joint_state_adapter.py",
                name="pzs100_rviz_joint_state_adapter",
                parameters=[{"use_sim_time": True}],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("crane_bringup"))
                / "launch"
                / "sim.launch.py",
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "world": LaunchConfiguration("gazebo_world_file"),
                    "joint_states_topic": "joint_states_rviz",
                    # One server per service: crane_planner stands down when
                    # the legacy one is started below.
                    "start_planner": use_native_a2b,
                    "start_supervisor": "false",
                }.items(),
            ),
            # ── The retained A2B planner ────────────────────────────────
            # `a2b_ilqr_server` reads the crane out of two latched
            # descriptions and refuses to answer until it has both:
            # `robot_description_full` -- which `sim.launch.py`'s
            # `robot_state_publisher` already publishes, the same Gazebo-baked
            # variant the timber twin feeds it -- and `crane_tools_description`,
            # which nothing in the new stack publishes. Hence the publisher
            # below; it is the tool half of the same pair, and it is started
            # only on this path.
            #
            # It subscribes to the raw `/joint_states` and not to the corrected
            # `/joint_states_rviz`, exactly as the timber twin does: the q9
            # EPSCOPE `state_factor` is in both the planner's start state and
            # the trajectory the controller follows, so the two agree.
            Node(
                package="crane_tools_description",
                executable="crane_tools_description_publisher",
                name="crane_tools_description_publisher",
                parameters=[
                    PathSubstitution(FindPackageShare("pzs100_description"))
                    / "config"
                    / "gripper_parameter.yaml",
                    {"use_sim_time": True},
                ],
                condition=IfCondition(use_legacy_a2b),
            ),
            Node(
                package="timber_crane_motion_planning",
                executable="a2b_ilqr_server",
                output="both",
                parameters=[
                    mp_param_path,
                    a2b_param_path,
                    {"use_sim_time": True},
                    # The rate `trajectory_controller_a2b` is fed at, as
                    # `mp.launch.py` sets it. Left off, the server upsamples to
                    # its own default.
                    {"a2bOptions": {"dtTarget": 0.01}},
                ],
                condition=IfCondition(use_legacy_a2b),
            ),
            # Publishes /collision_objects, the static site geometry the planner
            # avoids. Without it the server plans against the config's obstacles
            # alone -- the same arrangement, and the same file, as mp.launch.py.
            Node(
                package="collision_body_handler",
                executable="collision_body_handler",
                output="both",
                parameters=[
                    {"collision_objects_file": collision_objects_path},
                    {"use_sim_time": True},
                ],
                condition=IfCondition(use_legacy_a2b),
            ),
            # ── Deadman bridge (headless only) ──────────────────────────
            # sim.launch.py's own sim_remote publishes /crane/remote_ctrl_states
            # for crane_supervisor.  The BT reads the epsilon name instead, so a
            # second publisher is remapped onto it.  Two independent publishers
            # on two topics -- no relay, no edit to either stack.
            #
            # It runs only when the TUI does not.  sim_remote holds button12
            # true forever; the TUI below publishes the operator's real button
            # on the same topic, all-false whenever no key is held.  Both at
            # once and the BT's subscriber sees the two interleaved: the rising
            # edge CheckUserApproval waits for always arrives from sim_remote,
            # so every step self-approves, and GetUserApproval -- a deadman that
            # re-evaluates the latest message on every tick -- aborts the
            # trajectory on the first false, roughly 100 ms in.
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
                condition=UnlessCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_tui"),
                            "'.lower() in ('true', '1', 'yes')",
                        ]
                    )
                ),
            ),
            # ── RViz ────────────────────────────────────────────────────
            # cbs.rviz already has an enabled Path display on
            # /crane_planner/planned_path, which only this stack fills.
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                    / "rviz"
                    / "cbs.rviz",
                ],
                parameters=[{"use_sim_time": True}],
                # Started exactly as the timber twin starts it, which is the
                # arrangement the RViz 2D Goal Pose tool is known to work under.
                # No `name=`: that puts `-r __node:=rviz2` on the command line and
                # renames the node RViz builds its tool and panel plugins against.
                # No `output="log"` either -- a plugin that fails to load says so
                # on stderr, and this config lost its goal-pose tool to exactly
                # that, silently, for as long as the errors went to a file.
                condition=IfCondition(LaunchConfiguration("gui")),
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
                    cbs_stack_bt_config,
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
            # Own terminal window: this is where the operator approves a step.
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
