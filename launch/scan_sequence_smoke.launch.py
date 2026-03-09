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
            "params_file": motion_planning_params_file,
            "named_configurations_file": named_configurations_file,
            "wall_plan_file": wall_plan_file,
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

