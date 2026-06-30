"""PZS100 Gazebo experiment: place a 3-block row with the middle block last."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PathSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("gazebo_world_file", default_value="epsilon_crane.world"),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="1",
                description="Initial crane pose (see epsilon_crane_description)",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="pid",
                description="Low-level A2B controller: 'pid' or 'mpc'",
            ),
            DeclareLaunchArgument(
                "seed_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_world_model"),
                        "config",
                        "world_model_seed_truck_loading.yaml",
                    ]
                ),
                description="World-model/Gazebo seed YAML with world_model.initial_blocks.",
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("concrete_block_behavior_tree"))
                / "launch"
                / "gazebo_wall_assembly_pzs100.launch.py",
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "controller": LaunchConfiguration("controller"),
                    "place_approach_angle_deg": "0.0",
                    "lift_height": "1.2",
                    "seed_file": LaunchConfiguration("seed_file"),
                }.items(),
            ),
        ]
    )
