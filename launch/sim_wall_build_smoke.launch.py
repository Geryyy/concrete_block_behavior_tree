from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # Fast planner/simulation smoke profile only. This intentionally uses the
            # dummy BT path instead of the perception-backed scan tree.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("concrete_block_behavior_tree"), "launch", "sim_wall_build.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_perception": "False",
                    "concrete_rviz": "True",
                    "cleanup_stale_gazebo": "True",
                    "tool": "pzs100_description",
                }.items(),
            )
        ]
    )
