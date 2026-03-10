from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    initial_pose = LaunchConfiguration("initial_pose")
    tool = LaunchConfiguration("tool")

    use_perception = LaunchConfiguration("use_perception")
    concrete_rviz = LaunchConfiguration("concrete_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    start_bt_action_server = LaunchConfiguration("start_bt_action_server")
    bt_params_file = LaunchConfiguration("bt_params_file")
    bt_start_delay_s = LaunchConfiguration("bt_start_delay_s")
    gazebo_master_port = LaunchConfiguration("gazebo_master_port")

    gazebo_bringup = IncludeLaunchDescription(
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
            "rviz": "False",
            "joystick": "False",
            "matlab_excel": "False",
            "tool": tool,
            "gui": gui,
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

    bt_launch_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_bt_action_server": start_bt_action_server,
            "bt_params_file": bt_params_file,
        }.items(),
        condition=IfCondition(use_perception),
    )

    bt_launch_dummy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_bt_action_server": start_bt_action_server,
            "bt_params_file": PathJoinSubstitution(
                [FindPackageShare("concrete_block_behavior_tree"), "config", "dummy_start.yaml"]
            ),
        }.items(),
        condition=UnlessCondition(use_perception),
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
        condition=IfCondition(use_perception),
    )

    local_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(concrete_rviz),
    )

    delayed_bt_launch_full = TimerAction(
        period=bt_start_delay_s,
        actions=[bt_launch_full],
    )

    delayed_bt_launch_dummy = TimerAction(
        period=bt_start_delay_s,
        actions=[bt_launch_dummy],
    )

    gazebo_master_uri = SetEnvironmentVariable(
        name="GAZEBO_MASTER_URI",
        value=PythonExpression(["'http://127.0.0.1:' + str(", gazebo_master_port, ")"]),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("tool", default_value="epsilon_7040_description"),
            DeclareLaunchArgument("gazebo_master_port", default_value="11346"),
            DeclareLaunchArgument("use_perception", default_value="False"),
            DeclareLaunchArgument("concrete_rviz", default_value="True"),
            DeclareLaunchArgument("start_bt_action_server", default_value="True"),
            DeclareLaunchArgument("bt_start_delay_s", default_value="8.0"),
            DeclareLaunchArgument(
                "bt_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_behavior_tree"), "config", "default.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("concrete_block_behavior_tree"), "rviz", "concrete_bt.rviz"]
                ),
            ),
            gazebo_master_uri,
            gazebo_bringup,
            motion_planning_launch,
            delayed_bt_launch_full,
            delayed_bt_launch_dummy,
            perception_launch,
            local_rviz,
        ]
    )
