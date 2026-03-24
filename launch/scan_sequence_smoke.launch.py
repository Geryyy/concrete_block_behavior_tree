from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    bt_params_file = LaunchConfiguration("bt_params_file")
    motion_planning_params_file = LaunchConfiguration("motion_planning_params_file")
    named_configurations_file = LaunchConfiguration("named_configurations_file")
    wall_plan_file = LaunchConfiguration("wall_plan_file")
    planner_backend = LaunchConfiguration("planner_backend")
    planner_timber_a2b_service = LaunchConfiguration("planner_timber_a2b_service")
    planner_timber_goal_frame = LaunchConfiguration("planner_timber_goal_frame")
    planner_timber_move_empty_target_z = LaunchConfiguration(
        "planner_timber_move_empty_target_z"
    )
    execution_enabled = LaunchConfiguration("execution_enabled")
    execution_backend = LaunchConfiguration("execution_backend")
    execution_trajectory_topic = LaunchConfiguration("execution_trajectory_topic")
    execution_action_name = LaunchConfiguration("execution_action_name")
    execution_result_timeout_s = LaunchConfiguration("execution_result_timeout_s")
    execution_switch_controller = LaunchConfiguration("execution_switch_controller")
    execution_switch_service = LaunchConfiguration("execution_switch_service")
    execution_activate_controller = LaunchConfiguration("execution_activate_controller")
    execution_deactivate_after_execution = LaunchConfiguration(
        "execution_deactivate_after_execution"
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_perception"), "launch", "perception.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_world_model": "true",
        }.items(),
    )

    motion_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_motion_planning"), "launch", "motion_planning.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "motion_planning_params_file": motion_planning_params_file,
            "planner_backend": planner_backend,
            "planner_timber_a2b_service": planner_timber_a2b_service,
            "planner_timber_goal_frame": planner_timber_goal_frame,
            "planner_timber_move_empty_target_z": planner_timber_move_empty_target_z,
            "named_configurations_file": named_configurations_file,
            "wall_plan_file": wall_plan_file,
            "execution_enabled": execution_enabled,
            "execution_backend": execution_backend,
            "execution_trajectory_topic": execution_trajectory_topic,
            "execution_action_name": execution_action_name,
            "execution_result_timeout_s": execution_result_timeout_s,
            "execution_switch_controller": execution_switch_controller,
            "execution_switch_service": execution_switch_service,
            "execution_activate_controller": execution_activate_controller,
            "execution_deactivate_after_execution": execution_deactivate_after_execution,
        }.items(),
    )

    bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bt_params_file": bt_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("planner_backend", default_value="concrete"),
            DeclareLaunchArgument(
                "planner_timber_a2b_service", default_value="a2b_movement"
            ),
            DeclareLaunchArgument(
                "planner_timber_goal_frame", default_value="K0_mounting_base"
            ),
            DeclareLaunchArgument(
                "planner_timber_move_empty_target_z", default_value="2.36"
            ),
            DeclareLaunchArgument(
                "bt_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_behavior_tree"), "config", "scan_smoke.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "motion_planning_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_motion_planning"), "config", "motion_planning.yaml"]
                ),
            ),
            DeclareLaunchArgument("execution_enabled", default_value="true"),
            DeclareLaunchArgument("execution_backend", default_value="topic"),
            DeclareLaunchArgument(
                "execution_trajectory_topic",
                default_value="/trajectory_controllers/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "execution_action_name",
                default_value="/trajectory_controller_a2b/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "execution_result_timeout_s", default_value="120.0"
            ),
            DeclareLaunchArgument(
                "execution_switch_controller", default_value="false"
            ),
            DeclareLaunchArgument(
                "execution_switch_service",
                default_value="/controller_manager/switch_controller",
            ),
            DeclareLaunchArgument(
                "execution_activate_controller",
                default_value="trajectory_controllers",
            ),
            DeclareLaunchArgument(
                "execution_deactivate_after_execution", default_value="true"
            ),
            DeclareLaunchArgument(
                "named_configurations_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_motion_planning"), "config", "named_configurations.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "wall_plan_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_motion_planning"), "motion_planning", "data", "wall_plans.yaml"]
                ),
            ),
            perception_launch,
            motion_planning_launch,
            bt_launch,
        ]
    )
