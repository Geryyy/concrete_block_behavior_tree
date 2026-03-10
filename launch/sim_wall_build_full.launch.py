from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("concrete_block_behavior_tree"), "launch", "sim_wall_build.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_perception": "True",
                    "concrete_rviz": "True",
                    "cleanup_stale_gazebo": "True",
                    "tool": "pzs100_description",
                    "bt_start_delay_s": "10.0",
                }.items(),
            )
        ]
    )
