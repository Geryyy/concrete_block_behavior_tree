from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    bag = LaunchConfiguration("bag")

    perception_rosbag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_perception"), "launch", "rosbag_block_world_model.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bag": bag,
            "perception_mode": "IDLE",
        }.items(),
    )

    motion_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_motion_planning"), "launch", "motion_planning.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "bag",
                default_value="/home/vscode/Documents/2025-12-17/pzs_crane_1_pickup",
            ),
            perception_rosbag_launch,
            motion_planning_launch,
            bt_launch,
        ]
    )
