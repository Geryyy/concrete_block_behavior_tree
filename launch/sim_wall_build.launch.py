from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    cleanup_stale_gazebo = LaunchConfiguration("cleanup_stale_gazebo")

    xacro_path = PathJoinSubstitution(
        [FindPackageShare("epsilon_crane_description"), "urdf", "timber_loader_AIT.urdf.xacro"]
    )
    world_path = PathJoinSubstitution(
        [FindPackageShare("testsite_description"), "worlds", "epsilon_crane.world"]
    )
    pzs100_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "ros2_control",
            "pzs100_trajectory_forward.ros2_control.yaml",
        ]
    )

    robot_description_gazebo = Command(
        [
            "xacro",
            " ",
            xacro_path,
            " ",
            "gazebo:=true",
            " ",
            "enable_logs_sim:=false",
            " ",
            "enable_zed2i_sim:=false",
            " ",
            "enable_ait_stereo_sim:=false",
            " ",
            "initial_pose:=",
            initial_pose,
            " ",
            "tool:=",
            tool,
            " ",
            "post_setup:=134",
        ]
    )
    robot_description_rviz = Command(
        [
            "xacro",
            " ",
            xacro_path,
            " ",
            "gazebo:=false",
            " ",
            "load_ros2_control:=false",
            " ",
            "initial_pose:=",
            initial_pose,
            " ",
            "tool:=",
            tool,
            " ",
            "post_setup:=134",
        ]
    )

    use_sim_time_param = {"use_sim_time": use_sim_time}

    robot_state_publisher_gazebo = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_gazebo},
            use_sim_time_param,
            {"publish_frequency": 100.0},
        ],
        remappings=[
            ("robot_description", "robot_description_full"),
            ("tf", "tf_gazebo"),
            ("tf_static", "tf_static_gazebo"),
        ],
        arguments=["--ros-args", "--log-level", "WARN"],
        output="screen",
    )

    robot_state_publisher_rviz = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_rviz",
        parameters=[
            {"robot_description": robot_description_rviz},
            use_sim_time_param,
            {"publish_frequency": 100.0},
        ],
        arguments=["--ros-args", "--log-level", "WARN"],
        output="screen",
    )

    crane_tools_description = Node(
        package="crane_tools_description",
        executable="crane_tools_description_publisher",
        name="crane_tools_description_publisher",
        parameters=[
            PathJoinSubstitution([FindPackageShare(tool), "config", "gripper_parameter.yaml"]),
            use_sim_time_param,
        ],
        output="screen",
    )

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world_path,
            "pause": "False",
            "gui": gui,
        }.items(),
    )

    spawn_crane = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description_full",
            "-entity",
            "crane_model",
            "-timeout",
            "60",
            "-x",
            "-0.0",
            "-y",
            "-6.0",
            "-z",
            "0",
            "-R",
            "0",
            "-Y",
            "3.14",
        ],
        output="screen",
    )

    static_world_floor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "floor"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "60", "joint_state_broadcaster"],
        output="screen",
    )

    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager-timeout",
            "60",
            "-p",
            pzs100_controller_params,
            "trajectory_controllers",
        ],
        output="screen",
    )

    trajectory_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[trajectory_controller],
        )
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
            PathJoinSubstitution([FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"])
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
            PathJoinSubstitution([FindPackageShare("concrete_block_behavior_tree"), "launch", "bt.launch.py"])
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

    delayed_bt_launch_full = TimerAction(period=bt_start_delay_s, actions=[bt_launch_full])
    delayed_bt_launch_dummy = TimerAction(period=bt_start_delay_s, actions=[bt_launch_dummy])

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("concrete_block_perception"), "launch", "perception.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "start_world_model": "true"}.items(),
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

    gazebo_master_uri = SetEnvironmentVariable(
        name="GAZEBO_MASTER_URI",
        value=PythonExpression(["'http://127.0.0.1:' + str(", gazebo_master_port, ")"]),
    )

    cleanup_gazebo_processes = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "pkill -x gzserver || true; pkill -x gzclient || true; sleep 1",
        ],
        output="screen",
        condition=IfCondition(cleanup_stale_gazebo),
    )

    gazebo_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_gazebo_processes,
            on_exit=[gazebo_world],
        ),
        condition=IfCondition(cleanup_stale_gazebo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            DeclareLaunchArgument("gazebo_master_port", default_value="11346"),
            DeclareLaunchArgument("cleanup_stale_gazebo", default_value="True"),
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
            cleanup_gazebo_processes,
            gazebo_after_cleanup,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
                ),
                launch_arguments={"world": world_path, "pause": "False", "gui": gui}.items(),
                condition=UnlessCondition(cleanup_stale_gazebo),
            ),
            robot_state_publisher_gazebo,
            robot_state_publisher_rviz,
            crane_tools_description,
            spawn_crane,
            static_world_floor,
            joint_state_broadcaster,
            trajectory_after_joint_state,
            motion_planning_launch,
            delayed_bt_launch_full,
            delayed_bt_launch_dummy,
            perception_launch,
            local_rviz,
        ]
    )
