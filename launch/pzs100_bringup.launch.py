from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    planner = LaunchConfiguration("planner")
    controller = LaunchConfiguration("controller")

    mp_launch_file = PythonExpression(
        ["'mp.launch.py' if '", planner, "' == 'ilqr' else 'mp_esdf.launch.py'"]
    )
    # PZS100 PID controller config now lives in CBS; the MPC config still
    # lives in timber_crane_unconstrained_mpc_controller_cpp.
    ctrl_package = PythonExpression(
        [
            "'concrete_block_behavior_tree' if '",
            controller,
            "' == 'pid' else 'timber_crane_unconstrained_mpc_controller_cpp'",
        ]
    )
    ctrl_config = PythonExpression(
        [
            "'ros2_control/crane_controller_hydraulic_a2b_jtc_pid_pzs100.ros2_control.yaml' if '",
            controller,
            "' == 'pid' else 'crane_controller_hydraulic_a2b_jtc_mpc_pzs100.ros2_control.yaml'",
        ]
    )

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner",
                default_value="ilqr",
                description="Planner: 'ilqr' or 'vpsto'",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="pid",
                description="Controller: 'pid' or 'mpc'",
            ),
            # Spawn the PZS100 joint-state adapter that publishes
            # /joint_states_rviz with the per-rail-corrected EPSCOPE q9.
            Node(
                package="concrete_block_behavior_tree",
                executable="pzs100_rviz_joint_state_adapter.py",
                name="pzs100_rviz_joint_state_adapter",
                parameters=[{"use_sim_time": True}],
                arguments=["--ros-args", "--log-level", "WARN"],
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
                    "mp_launch_file": mp_launch_file,
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
                    "enable_gripper_tip_publisher": "False",
                    "enable_pump_flow_estimator": "False",
                    "rviz_config": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "rviz"
                    / "cbs.rviz",
                    "joint_states_topic": "joint_states_rviz",
                }.items(),
            ),
        ]
    )

    # World model starts empty for Gazebo BT runs.
    # Blocks are spawned from the seed YAML, then upserted here from settled Gazebo poses.
    world_model_seed = (
        PathSubstitution(FindPackageShare("concrete_block_perception"))
        / "config"
        / "world_model_seed_none.yaml"
    )

    ld.add_action(
        Node(
            package="concrete_block_perception",
            executable="world_model_node",
            name="world_model_node",
            parameters=[
                PathSubstitution(FindPackageShare("concrete_block_perception"))
                / "config"
                / "world_model.yaml",
                world_model_seed,
                {
                    "use_sim_time": True,
                    "world_frame": "K0_mounting_base",
                    "pipeline_mode": "idle",
                    "perception_mode": "IDLE",
                },
            ],
            remappings=[
                ("block_world_model", "/cbp/block_world_model"),
                ("block_world_model_markers", "/cbp/block_world_model_markers"),
            ],
            output="screen",
        )
    )

    return ld
