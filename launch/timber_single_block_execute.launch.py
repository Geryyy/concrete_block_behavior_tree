from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    single_block_execute_profile = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "profiles",
            "single_block_execute.yaml",
        ]
    )
    bt_operator_config = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "bt_operator.yaml"]
    )
    gazebo_model_bt_launch = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "launch",
            "gazebo_model_bt.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_perception", default_value="true"),
            DeclareLaunchArgument("seed_world_model", default_value="true"),
            DeclareLaunchArgument("logs_sim", default_value="false"),
            DeclareLaunchArgument("zed2i_sim", default_value="false"),
            DeclareLaunchArgument("ait_stereo_sim", default_value="false"),
            DeclareLaunchArgument("enable_livox_sim", default_value="simple"),
            DeclareLaunchArgument("initial_pose", default_value="2"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_model_bt_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "gui": LaunchConfiguration("gui"),
                    "motion_backend": "timber",
                    "use_perception": LaunchConfiguration("use_perception"),
                    "seed_world_model": LaunchConfiguration("seed_world_model"),
                    "logs_sim": LaunchConfiguration("logs_sim"),
                    "zed2i_sim": LaunchConfiguration("zed2i_sim"),
                    "ait_stereo_sim": LaunchConfiguration("ait_stereo_sim"),
                    "enable_livox_sim": LaunchConfiguration("enable_livox_sim"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "tool": LaunchConfiguration("tool"),
                    "bt_mode_params_file": bt_operator_config,
                    "bt_profile_params_file": single_block_execute_profile,
                }.items(),
            ),
        ]
    )
