from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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

# The two values that start the standalone crane_mpc. They compose identically;
# they differ only in the node's `mode`.
CBS_MPC_CONTROLLERS = ("cbs_mpc", "cbs_mpc_active")


def reject_cbs_mpc_without_cbs_planner(context, *_args, **_kwargs):
    """
    Refuse the one controller/planner pair that would run without a reference.

    `crane_mpc` tracks `/crane/reference`, which only `crane_planning`'s
    `crane_planner` publishes. With a legacy A2B server upstream the node would
    start, configure on nothing and stay silent -- a failure that looks like a
    tuning problem. Raised here, before any other action runs, so nothing starts.
    """
    controller = context.launch_configurations["controller"]
    planner = context.launch_configurations["planner"]
    if controller in CBS_MPC_CONTROLLERS and planner != "cbs":
        raise RuntimeError(
            f"controller:={controller} requires planner:=cbs, got "
            f"planner:={planner}. The standalone crane_mpc tracks "
            "/crane/reference, which only the cbs planner publishes; the legacy "
            "A2B servers publish none."
        )
    return None


def generate_launch_description():
    planner = LaunchConfiguration("planner")
    controller = LaunchConfiguration("controller")
    seed_file = LaunchConfiguration("seed_file")
    grasp_detector_config = (
        PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
        / "config"
        / "gripper_grasp_detector_sim.yaml"
    )

    mp_launch_file = PythonExpression(
        ["'mp.launch.py' if '", planner, "' == 'ilqr' else 'mp_esdf.launch.py'"]
    )
    # 'cbs' takes the mp_esdf route purely to get its no-A2B-server branch: with
    # this backend neither legacy server nor the ESDF mapping starts, and the
    # crane_planner included below is the sole owner of /a2b_movement.
    mp_backend = PythonExpression(["'cbs' if '", planner, "' == 'cbs' else 'timber'"])
    # PZS100 PID and MPC controller configs both live in CBS. Only 'mpc' selects
    # the in-JTC JtcMpcPlugin: the two 'cbs_mpc*' values run the PID JTC and put
    # the standalone crane_mpc beside it, and two MPCs in one launch is the
    # failure 107 names.
    ctrl_package = "concrete_block_behavior_tree"
    ctrl_config = PythonExpression(
        [
            "'ros2_control/crane_controller_hydraulic_a2b_jtc_mpc_pzs100.ros2_control.yaml' if '",
            controller,
            "' == 'mpc' else 'ros2_control/crane_controller_hydraulic_a2b_jtc_pid_pzs100.ros2_control.yaml'",
        ]
    )

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner",
                default_value="ilqr",
                choices=["ilqr", "vpsto", "cbs"],
                description=(
                    "A2B planner: 'ilqr'/'vpsto' are the legacy timber servers, "
                    "'cbs' is crane_planning's crane_planner on the same "
                    "/a2b_movement service"
                ),
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
            # Off by default, because the behaviour tree is the other caller and
            # it sends no goal at all on this profile: issue 148 skips the
            # FollowJointTrajectory action whenever the MPC drives, so a gate on
            # by default would leave a tree that plans correctly and never moves.
            DeclareLaunchArgument(
                "mpc_waits_for_start",
                default_value="False",
                choices=["True", "False"],
                description=(
                    "Hold the standalone crane_mpc until the a2b trajectory "
                    "controller is given a goal, which is what the RViz motion "
                    "planning panel's 'Start trajectory' button sends. Without "
                    "it the planner's own publication of /crane/reference "
                    "starts the motion, so the machine leaves on 'Calculate'. "
                    "For a human at the panel; leave it False for the tree."
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
            DeclareLaunchArgument(
                "gui",
                default_value="True",
                description="Flag to launch gazebo+rviz2 GUI.",
            ),
            DeclareLaunchArgument(
                "gazebo_world_file",
                default_value="epsilon_crane.world",
                description=(
                    "World file from testsite_description/worlds. Use empty.world "
                    "for a clean sim without the Seibersdorf container."
                ),
            ),
            DeclareLaunchArgument(
                "enable_livox_sim",
                default_value="off",
                description="Enable the raw PointCloud2 Livox simulation with value 'livox'.",
            ),
            DeclareLaunchArgument(
                "start_grasp_detector",
                default_value="True",
                description="Start the q9 gripper grasp detector for PZS100 Gazebo runs.",
            ),
            # First action after the declarations, so an unusable combination
            # stops the launch before anything is spawned.
            OpaqueFunction(function=reject_cbs_mpc_without_cbs_planner),
            Node(
                package="concrete_block_behavior_tree",
                executable="gripper_grasp_detector",
                name="gripper_grasp_detector",
                output="screen",
                parameters=[grasp_detector_config, {"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_grasp_detector")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_sim"))
                / "launch"
                / "gazebo_model_bt.launch.py",
                launch_arguments={
                    "tool": "pzs100_description",
                    "controller_common_config_package": "concrete_block_behavior_tree",
                    "controller_common_config": "crane_controller_hydraulic_common_pzs100.ros2_control.yaml",
                    "controller_a2b_config_package": ctrl_package,
                    "controller_a2b_config": ctrl_config,
                    # `mpc` is misnamed: its only use in gazebo_model_bt.launch.py
                    # is UnlessCondition on the one trajectory_controller_a2b
                    # spawner, i.e. mpc:=True spawns no A2B controller at all. Both
                    # configs are read by that same spawner, so ctrl_config above is
                    # the whole switch. Undeclared there, so it must still be passed.
                    "mpc": "False",
                    "mp_launch_file": mp_launch_file,
                    "mp_backend": mp_backend,
                    "mp_param_file": "mp_parameter_pzs100.yaml",
                    "mp_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config/motion_planning/mp_parameter_pzs100.yaml",
                    "grip_traj_param_file": "grip_traj_parameter_pzs100.yaml",
                    "grip_traj_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config/motion_planning/grip_traj_parameter_pzs100.yaml",
                    "estimators_param_file": "estimators_pzs100.yaml",
                    "estimators_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config/estimators_pzs100.yaml",
                    "enable_gripper_tip_publisher": "False",
                    "enable_pump_flow_estimator": "False",
                    "enable_log_state_estimator": "False",
                    "rviz_config": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "rviz"
                    / "cbs.rviz",
                    "gui": LaunchConfiguration("gui"),
                    "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
                    "enable_livox_sim": LaunchConfiguration("enable_livox_sim"),
                }.items(),
            ),
        ]
    )

    # World model is seeded from the same YAML the Gazebo block spawner uses, so
    # markers / GetCoarseBlocks and the spawned Gazebo blocks share one source.
    ld.add_action(
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
                    # Whole CBS stack (spec, plan, world model, viz) stays in `world`;
                    # the single world -> K0_mounting_base conversion happens in the
                    # GetNextAssemblyTask BT plugin.
                    "world_frame": "world",
                    # refine_grasped.tcp_frame no longer overridden: world_model.yaml names
                    # the rigid TCP the BT takes the grasp offset against. Two copies of it
                    # are how hardware ended up on the other frame.
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
        )
    )

    # The native planner, when it is the one selected. It serves the same
    # /a2b_movement the legacy server did, so nothing above the planner changes.
    ld.add_action(
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare("crane_planning"))
            / "launch"
            / "crane_planner.launch.py",
            launch_arguments={
                "use_sim_time": "true",
                # JtcMpcPlugin never reads desired.effort -- it feeds forward from
                # its own actuator model -- and the mpc JTC config does not set
                # `effort_field_is_feedforward`, so a plan carrying C3's inversion
                # is rejected at goal acceptance. Drop it on that profile only.
                "c3_feedforward": PythonExpression(["'", controller, "' != 'mpc'"]),
            }.items(),
            condition=IfCondition(PythonExpression(["'", planner, "' == 'cbs'"])),
        )
    )

    # The standalone MPC, on the two 'cbs_mpc*' values. On 'cbs_mpc' it stays in
    # shadow (124): the chain is wired and judged on /crane/mpc/solver_health and
    # the contract topic is silent. On 'cbs_mpc_active' the same composition puts
    # it in active mode, and the a2b spawner's remap of the JTC's
    # ~/joint_trajectory onto /crane/mpc/horizon is what makes the horizon drive.
    # Until issue 148 takes the action goal out of the tree, that profile has
    # *two* commanders live -- bench configuration, not a deployment.
    # joint_states_topic is left at its default: this profile runs the
    # broadcaster's /joint_states uncorrected, so there is no corrected topic to
    # point the node at.
    starts_standalone_mpc = PythonExpression(
        ["'", controller, "' in ", str(CBS_MPC_CONTROLLERS)]
    )
    ld.add_action(
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare("crane_mpc"))
            / "launch"
            / "crane_mpc.launch.py",
            launch_arguments={
                "use_sim_time": "true",
                "mode": PythonExpression(
                    ["'active' if '", controller, "' == 'cbs_mpc_active' else 'shadow'"]
                ),
                # This profile's a2b spawner remaps the JTC's ~/controller_state
                # onto the shared /trajectory_controllers/controller_state, not
                # onto the contract name the node defaults to, so shadow mode
                # would have nothing to compare against. Shared with the
                # grasping controller; the node drops what does not name its
                # joints.
                "controller_state_topic": "/trajectory_controllers/controller_state",
                # The panel's Start button sends its goal here and nowhere else
                # (mp_rviz_panel.hpp:210 hard-codes the name), so this is the
                # only name the gate can watch.
                "start_signal_action": PythonExpression(
                    [
                        "'/trajectory_controller_a2b/follow_joint_trajectory' if '",
                        LaunchConfiguration("mpc_waits_for_start"),
                        "' == 'True' else ''",
                    ]
                ),
            }.items(),
            condition=IfCondition(starts_standalone_mpc),
        )
    )

    return ld
