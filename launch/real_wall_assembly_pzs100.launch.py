"""Real PZS100 wall-assembly operator stack, without Gazebo.

This starts only ROS-side infrastructure for the real crane: description/TF,
motion-planning servers, point-cloud block detection + world model, wall-plan server, BT server,
and RViz. The low-level ros2_control bridge is usually managed outside this
launch on the crane IPC, but controller spawners can be enabled with
start_controller_spawners:=true.
"""

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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    tool = LaunchConfiguration("tool")
    planner = LaunchConfiguration("planner")
    seed_block_0 = LaunchConfiguration("seed_block_0")

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
        / "gripper_grasp_detector_real.yaml"
    )
    mp_launch_file = PythonExpression(
        ["'mp.launch.py' if '", planner, "' == 'ilqr' else 'mp_esdf.launch.py'"]
    )
    world_model_seed_block_0 = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_world_model"),
            "config",
            "world_model_seed_b0.yaml",
        ]
    )
    selected_world_model_overlay_params_file = PythonExpression(
        [
            "'",
            world_model_seed_block_0,
            "' if '",
            seed_block_0,
            "'.lower() in ('true', '1', 'yes', 'on') else '",
            LaunchConfiguration("world_model_overlay_params_file"),
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("post_setup", default_value="13"),
            DeclareLaunchArgument(
                "planner",
                default_value="ilqr",
                description=(
                    "Planner: 'ilqr' uses mp.launch.py; anything else uses "
                    "mp_esdf.launch.py."
                ),
            ),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("start_estimators", default_value="true"),
            DeclareLaunchArgument(
                "start_controller_spawners",
                default_value="false",
                description=(
                    "Spawn common/hardware controllers if the real bridge is "
                    "already available."
                ),
            ),
            DeclareLaunchArgument("start_motion_planning", default_value="true"),
            DeclareLaunchArgument(
                "start_perception",
                default_value="false",
                description="Start the point-cloud detector and its world-model bridge.",
            ),
            DeclareLaunchArgument(
                "start_world_model",
                default_value="true",
                description="Start the world model within the detector pipeline.",
            ),
            DeclareLaunchArgument("start_wall_plan_server", default_value="true"),
            DeclareLaunchArgument("start_bt_action_server", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("points_topic", default_value="/seyond/points"),
            DeclareLaunchArgument(
                "world_model_overlay_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_world_model"),
                        "config",
                        "world_model_seed_none.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "seed_block_0",
                default_value="false",
                description=(
                    "Testing helper: seed block_0 into the world model at startup. "
                    "Leave false when perception should populate the model."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_behavior_tree"),
                        "rviz",
                        "cbs.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "bt_xml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lsrl_behavior_tree"),
                        "behavior_trees",
                        "empty.xml",
                    ]
                ),
                description="Initial tree loaded by bt_action_server; RViz panel can switch it.",
            ),
            DeclareLaunchArgument(
                "place_approach_angle_deg",
                default_value="4.0",
                description="Pre-place lateral approach angle in degrees.",
            ),
            DeclareLaunchArgument(
                "lift_height",
                default_value="1.0",
                description="Vertical post-grasp/post-place lift height for CalcGripMovement.",
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
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_common"))
                / "launch"
                / "description.launch.py",
                launch_arguments={
                    "tool": tool,
                    "fake_hardware": "false",
                    "bag_replay": "false",
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "post_setup": LaunchConfiguration("post_setup"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_description")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_common"))
                / "launch"
                / "estimators.launch.py",
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "tool": tool,
                    "estimators_param_file": "estimators_pzs100.yaml",
                    "estimators_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config"
                    / "estimators_pzs100.yaml",
                    "enable_gripper_tip_publisher": "false",
                    "enable_pump_flow_estimator": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_estimators")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_common"))
                / "launch"
                / "common_controllers.launch.py",
                launch_arguments={
                    "tool": tool,
                    "enable_log_state_estimator": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_controller_spawners")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_common"))
                / "launch"
                / "hardware_controllers.launch.py",
                condition=IfCondition(LaunchConfiguration("start_controller_spawners")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("epsilon_crane_bringup_mp"))
                / "launch"
                / mp_launch_file,
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "mp_param_file": "mp_parameter_pzs100.yaml",
                    "mp_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config"
                    / "motion_planning"
                    / "mp_parameter_pzs100.yaml",
                    "grip_traj_param_file": "grip_traj_parameter_pzs100.yaml",
                    "grip_traj_param_path": PathSubstitution(
                        FindPackageShare("concrete_block_behavior_tree")
                    )
                    / "config"
                    / "motion_planning"
                    / "grip_traj_parameter_pzs100.yaml",
                    "start_grip_traj_server": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_motion_planning")),
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
                    {
                        "lift_height": ParameterValue(
                            LaunchConfiguration("lift_height"),
                            value_type=float,
                        ),
                        "use_sim_time": use_sim_time,
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_motion_planning")),
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("concrete_block_detector"))
                / "launch"
                / "wall_assembly_perception.launch.py",
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world_model_overlay_params_file": selected_world_model_overlay_params_file,
                    "start_world_model": LaunchConfiguration("start_world_model"),
                    "points_topic": LaunchConfiguration("points_topic"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_perception")),
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
                        "use_sim_time": use_sim_time,
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
                condition=IfCondition(LaunchConfiguration("start_wall_plan_server")),
            ),
            Node(
                package="concrete_block_behavior_tree",
                executable="gripper_grasp_detector",
                name="gripper_grasp_detector",
                output="screen",
                parameters=[
                    grasp_detector_config,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
            Node(
                package="lsrl_behavior_tree",
                executable="bt_action_server",
                output="both",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": use_sim_time},
                    {"behaviortree": LaunchConfiguration("bt_xml")},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[
                    base_bt_config,
                    override_bt_config,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(LaunchConfiguration("start_bt_action_server")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_real_wall_assembly_pzs100",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
