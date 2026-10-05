"""
PZS100 Gazebo simulation with CBS wall assembly behavior tree.

Launches the full PZS100 crane simulation (without the epsilon_crane BT)
and adds the CBS wall assembly pipeline: wall_plan_server, world_model,
and a bt_action_server with CBS + epsilon_crane plugins.
"""

import os

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PathSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
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
    seed_file = LaunchConfiguration("seed_file")

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
            DeclareLaunchArgument(
                "gazebo_world_file",
                default_value="epsilon_crane.world",
                description=(
                    "World file from testsite_description/worlds. Use empty.world "
                    "for a clean sim without the Seibersdorf container."
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="1",
                description="Initial crane pose (see epsilon_crane_description)",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="pid",
                choices=["pid", "mpc", "cbs_mpc", "cbs_mpc_active"],
                description=(
                    "Low-level A2B controller: 'pid' is the JTC with the PID "
                    "plugin; 'mpc' is the same JTC with the in-process "
                    "JtcMpcPlugin; 'cbs_mpc' keeps the PID JTC and starts the "
                    "standalone crane_mpc node beside it, in shadow mode; "
                    "'cbs_mpc_active' composes identically but puts that node "
                    "in active mode, so its horizon drives the JTC. Both "
                    "cbs_mpc values require planner:=cbs"
                ),
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="ilqr",
                choices=["ilqr", "vpsto", "cbs"],
                description=(
                    "A2B planner: 'ilqr'/'vpsto' are the legacy timber servers, "
                    "'cbs' is crane_planning's crane_planner"
                ),
            ),
            DeclareLaunchArgument(
                "mpc_waits_for_start",
                default_value="False",
                choices=["True", "False"],
                description=(
                    "Forwarded to pzs100_bringup: hold the standalone crane_mpc "
                    "until the a2b trajectory controller is given a goal, which "
                    "is what the RViz panel's 'Start trajectory' button sends. "
                    "For a human driving from the panel. Leave it False for the "
                    "tree, which sends no goal while the MPC drives and gates "
                    "the move on GetUserApproval activating the controller "
                    "instead (cbs_subtree_a2b_movement.xml)."
                ),
            ),
            DeclareLaunchArgument(
                "place_approach_angle_deg",
                # 18: the place waypoint sits 0.30 tan(18) = 0.10 m to the open side; 4 (2 cm)
                # left no clearance to a neighbour 2 cm away (sim wall run 65).
                default_value="18.0",
                description="Pre-place lateral approach angle in degrees.",
            ),
            DeclareLaunchArgument(
                "lift_height",
                # 1.0 carried the block over its neighbour with ~0.3 m to spare and the
                # MPC sagged 0.18 m crossing it: the corner caught (sim run 17).
                default_value="1.5",
                description="Vertical post-grasp/post-place lift height for CalcGripMovement.",
            ),
            DeclareLaunchArgument(
                "start_perception",
                default_value="false",
                description="Start the concrete-block detector on the simulated raw LiDAR cloud.",
            ),
            DeclareLaunchArgument(
                "lidar_points_topic",
                default_value="/seyond/points",
                description="Raw PointCloud2 topic of the simulated Seyond (SeyondLidar gz system).",
            ),
            DeclareLaunchArgument(
                "enable_placement_disturbance",
                default_value="false",
                description=(
                    "Inject a one-shot initial-hover pose bias in the operator workflow, "
                    "so the placement correction loop can be tested."
                ),
            ),
            DeclareLaunchArgument("placement_disturbance_x_m", default_value="0.06"),
            DeclareLaunchArgument("placement_disturbance_y_m", default_value="-0.04"),
            DeclareLaunchArgument("placement_disturbance_z_m", default_value="0.00"),
            DeclareLaunchArgument(
                "placement_disturbance_yaw_rad", default_value="0.035"
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
            # Vision start: world_model_seed_none.yaml here, and the blocks Gazebo spawns
            # from seed_file are found by SCENE_DISCOVERY instead of known up front.
            DeclareLaunchArgument("world_model_seed_file", default_value=seed_file),
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
            # The panel's "Move empty" button is fixed to one tree per profile.
            # Its default is epsilon's `move_empty.xml`, which this package does
            # not have, so without this the button stays greyed out and the only
            # way to an A2B move is the dropdown -- where the neighbouring CBS
            # entry needs a launch profile this one is not.
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_MOVE_EMPTY",
                value="/behavior_trees/move_empty_pzs100.xml",
            ),
            # ── PZS100 crane simulation (without epsilon_crane BT) ───────
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                / "launch"
                / "pzs100_bringup.launch.py",
                launch_arguments={
                    "start_bt_action_server": "False",
                    "start_grip_traj_server": "False",
                    "controller": LaunchConfiguration("controller"),
                    "planner": LaunchConfiguration("planner"),
                    "mpc_waits_for_start": LaunchConfiguration("mpc_waits_for_start"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "seed_file": LaunchConfiguration("world_model_seed_file"),
                    "gui": LaunchConfiguration("gui"),
                    "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
                }.items(),
            ),
            # The PZS100 bringup owns the seeded world model.  The detector
            # talks to it through /concrete_block_detector/discover_blocks;
            # launching the detector directly avoids a second world model.
            # CPU Seyond lidar (no GPU): the SeyondLidar gz system, attached to the
            # running world, ray-casts every collision shape; bridged to ROS.
            ExecuteProcess(
                cmd=[
                    "until gz service -s /world/default/entity/system/add"
                    " --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 2000"
                    ' --req \'entity: {name: "default", type: WORLD} plugins: {name:'
                    ' "concrete_block_behavior_tree::SeyondLidar", filename: "SeyondLidarSystem",'
                    " innerxml: \"<topic>/seyond/points</topic>\"}' | grep -q 'data: true';"
                    " do sleep 1; done"
                ],
                shell=True,
                name="seyond_lidar_attach",
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="seyond_lidar_bridge",
                arguments=[
                    "/seyond/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
                ],
                remappings=[
                    ("/seyond/points", LaunchConfiguration("lidar_points_topic"))
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_perception")),
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
                        # Sim publishes the raw cloud; detector.yaml defaults to
                        # cloudini for hardware.  Debug outlets are configured in
                        # detector.yaml, not here.
                        "point_cloud_transport": "raw",
                        # detector.yaml names the hardware estimator's elastic/
                        # frames; the sim publishes the rigid ones.
                        "pose_priors.fk.tcp_frame": "K8_tool_center_point",
                        "gripper_self_filter.left_rail.parent_frame": "K9",
                        "gripper_self_filter.left_rail.rail_frame": "K10_left_rail",
                        "gripper_self_filter.right_rail.parent_frame": "K11",
                        "gripper_self_filter.right_rail.rail_frame": "K12_right_rail",
                    },
                ],
                remappings=[("points", LaunchConfiguration("lidar_points_topic"))],
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            # World model is launched by gazebo_model_bt_pzs100.launch.py.
            # Virtual TCP TF is published by grip_traj_server from tcp_z_offset param.
            # ── Simple grip trajectory server ────────────────────────────
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
            # ── Wall plan server (assembly planning layer) ───────────────
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
                        # One commander per move (issue 148). `cbs_mpc_active` is
                        # the only profile where crane_mpc re-goals the JTC by
                        # topic, so it is the only one where the CBS A2B subtree
                        # drops its FollowJointTrajectory goal and waits on
                        # WaitForMoveComplete instead.
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
                            FindPackageShare("lsrl_behavior_tree")
                        )
                        / "behavior_trees"
                        / "empty.xml"
                    },
                ],
            ),
            # ── Lifecycle manager ────────────────────────────────────────
            # Reads lifecycle_manager params (node_names, autostart, …) from
            # bt_server_override.yaml (CBS-owned, since upstream removed them
            # from epsilon_crane_behavior_tree/default.yaml).
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
            # ── Gazebo block spawner ─────────────────────────────────────
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
                                # a -6 m y shift. (Reproduces the K0 calibration:
                                # K0-in-gazebo = world->gazebo applied to T_world_K0.)
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
                    # Stands in for the detector when start_perception is off: true block
                    # poses + noise on the detector's service, for the world model's REFINE_*.
                    Node(
                        package="concrete_block_behavior_tree",
                        executable="gazebo_block_oracle.py",
                        name="concrete_block_detector",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                        condition=UnlessCondition(
                            LaunchConfiguration("start_perception")
                        ),
                    ),
                ],
            ),
            # ── Keyboard TUI ────────────────────────────────────────────
            # Also the sim's operator-approval producer: it publishes
            # /gpio_controller/remote_ctrl_states itself, so `z` (button12) held
            # in its xterm is what GetUserApproval/CheckUserApproval read here.
            # Dropping it, or gui:=False, leaves every approval node blocking.
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
