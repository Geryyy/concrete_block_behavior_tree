from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    initial_pose = LaunchConfiguration("initial_pose")
    tool = LaunchConfiguration("tool")

    gazebo_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("epsilon_crane_bringup_sim"), "launch", "gazebo_model_manual.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "logs_sim": "False",
            "zed2i_sim": "False",
            "ait_stereo_sim": "False",
            "initial_pose": initial_pose,
            "sw_controller": "1",
            "plot": "False",
            "rviz": "True",
            "joystick": "False",
            "matlab_excel": "False",
            "tool": tool,
            "gui": gui,
        }.items(),
    )

    bt_dummy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_bt_action_server": "true",
            "bt_params_file": PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "config", "dummy_start.yaml"]
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("tool", default_value="epsilon_7040_description"),
            gazebo_and_rviz,
            bt_dummy,
        ]
    )
