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
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    start_paused = LaunchConfiguration("start_paused")
    auto_unpause = LaunchConfiguration("auto_unpause")
    unpause_delay_s = LaunchConfiguration("unpause_delay_s")
    initial_pose = LaunchConfiguration("initial_pose")
    tool = LaunchConfiguration("tool")
    spawn_concrete_block = LaunchConfiguration("spawn_concrete_block")
    concrete_block_name = LaunchConfiguration("concrete_block_name")
    concrete_block_x = LaunchConfiguration("concrete_block_x")
    concrete_block_y = LaunchConfiguration("concrete_block_y")
    concrete_block_z = LaunchConfiguration("concrete_block_z")
    concrete_block_yaw = LaunchConfiguration("concrete_block_yaw")
    cbmp_execution_enabled = LaunchConfiguration("cbmp_execution_enabled")
    cbmp_execution_backend = LaunchConfiguration("cbmp_execution_backend")
    cbmp_execution_topic = LaunchConfiguration("cbmp_execution_topic")
    cbmp_default_trajectory_method = LaunchConfiguration(
        "cbmp_default_trajectory_method"
    )
    cbmp_execution_action_name = LaunchConfiguration("cbmp_execution_action_name")
    cbmp_execution_switch_controller = LaunchConfiguration(
        "cbmp_execution_switch_controller"
    )
    cbmp_execution_activate_controller = LaunchConfiguration(
        "cbmp_execution_activate_controller"
    )
    enable_rviz_move_empty_interface = LaunchConfiguration(
        "enable_rviz_move_empty_interface"
    )
    rviz_move_empty_goal_topic = LaunchConfiguration("rviz_move_empty_goal_topic")
    rviz_move_empty_world_frame = LaunchConfiguration("rviz_move_empty_world_frame")
    rviz_move_empty_tool_frame = LaunchConfiguration("rviz_move_empty_tool_frame")
    rviz_move_empty_enable_topic = LaunchConfiguration("rviz_move_empty_enable_topic")

    use_perception = LaunchConfiguration("use_perception")
    concrete_rviz = LaunchConfiguration("concrete_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    start_bt_action_server = LaunchConfiguration("start_bt_action_server")
    bt_params_file = LaunchConfiguration("bt_params_file")
    bt_start_delay_s = LaunchConfiguration("bt_start_delay_s")
    gazebo_master_port = LaunchConfiguration("gazebo_master_port")
    cleanup_stale_gazebo = LaunchConfiguration("cleanup_stale_gazebo")

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_description"),
            "urdf",
            "timber_loader_AIT.urdf.xacro",
        ]
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
    concrete_block_model_path = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "models",
            "concrete_block.sdf",
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
            PathJoinSubstitution(
                [FindPackageShare(tool), "config", "gripper_parameter.yaml"]
            ),
            use_sim_time_param,
        ],
        output="screen",
    )

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": world_path,
            "pause": start_paused,
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

    spawn_concrete_block_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            concrete_block_model_path,
            "-entity",
            concrete_block_name,
            "-timeout",
            "60",
            "-x",
            concrete_block_x,
            "-y",
            concrete_block_y,
            "-z",
            concrete_block_z,
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            concrete_block_yaw,
        ],
        output="screen",
        condition=IfCondition(spawn_concrete_block),
    )

    spawn_block_after_crane = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_crane,
            on_exit=[spawn_concrete_block_node],
        ),
        condition=IfCondition(spawn_concrete_block),
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

    should_unpause = IfCondition(
        PythonExpression([start_paused, " and ", auto_unpause])
    )
    unpause_physics = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
        condition=should_unpause,
    )
    delayed_unpause = TimerAction(period=unpause_delay_s, actions=[unpause_physics])
    unpause_after_block_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_concrete_block_node,
            on_exit=[delayed_unpause],
        ),
        condition=IfCondition(spawn_concrete_block),
    )
    unpause_after_trajectory = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_controller,
            on_exit=[delayed_unpause],
        ),
        condition=UnlessCondition(spawn_concrete_block),
    )

    motion_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("concrete_block_motion_planning"),
                    "launch",
                    "motion_planning.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "default_trajectory_method": cbmp_default_trajectory_method,
            "execution_enabled": cbmp_execution_enabled,
            "execution_backend": cbmp_execution_backend,
            "execution_trajectory_topic": cbmp_execution_topic,
            "execution_action_name": cbmp_execution_action_name,
            "execution_switch_controller": cbmp_execution_switch_controller,
            "execution_activate_controller": cbmp_execution_activate_controller,
        }.items(),
    )

    rviz_move_empty_interface = Node(
        package="concrete_block_motion_planning",
        executable="rviz_move_empty_interface.py",
        name="rviz_move_empty_interface",
        output="screen",
        parameters=[
            {"goal_topic": rviz_move_empty_goal_topic},
            {"world_frame": rviz_move_empty_world_frame},
            {"tool_frame": rviz_move_empty_tool_frame},
            {"enable_topic": rviz_move_empty_enable_topic},
            {"dry_run": False},
        ],
        condition=IfCondition(enable_rviz_move_empty_interface),
    )

    bt_launch_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("concrete_block_behavior_tree"),
                    "launch",
                    "bt.launch.py",
                ]
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
                [
                    FindPackageShare("concrete_block_behavior_tree"),
                    "launch",
                    "bt.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_bt_action_server": start_bt_action_server,
            "bt_params_file": PathJoinSubstitution(
                [
                    FindPackageShare("concrete_block_behavior_tree"),
                    "config",
                    "dummy_start.yaml",
                ]
            ),
        }.items(),
        condition=UnlessCondition(use_perception),
    )

    delayed_bt_launch_full = TimerAction(
        period=bt_start_delay_s, actions=[bt_launch_full]
    )
    delayed_bt_launch_dummy = TimerAction(
        period=bt_start_delay_s, actions=[bt_launch_dummy]
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("concrete_block_perception"),
                    "launch",
                    "perception.launch.py",
                ]
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

    gazebo_master_uri = SetEnvironmentVariable(
        name="GAZEBO_MASTER_URI",
        value=PythonExpression(["'http://127.0.0.1:' + str(", gazebo_master_port, ")"]),
    )

    cleanup_gazebo_processes = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "while pgrep -x gzserver >/dev/null; do pkill -9 -x gzserver || true; sleep 0.2; done; "
                "while pgrep -x gzclient >/dev/null; do pkill -9 -x gzclient || true; sleep 0.2; done; "
                "sleep 1"
            ),
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

    delayed_runtime_bringup = TimerAction(
        period=3.0,
        actions=[
            robot_state_publisher_gazebo,
            robot_state_publisher_rviz,
            crane_tools_description,
            spawn_crane,
            spawn_block_after_crane,
            static_world_floor,
            joint_state_broadcaster,
            trajectory_after_joint_state,
            motion_planning_launch,
            rviz_move_empty_interface,
            delayed_bt_launch_full,
            delayed_bt_launch_dummy,
            perception_launch,
            local_rviz,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("gui", default_value="True"),
            DeclareLaunchArgument("start_paused", default_value="True"),
            DeclareLaunchArgument("auto_unpause", default_value="True"),
            DeclareLaunchArgument("unpause_delay_s", default_value="0.5"),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            DeclareLaunchArgument("spawn_concrete_block", default_value="True"),
            DeclareLaunchArgument(
                "concrete_block_name", default_value="concrete_block_1"
            ),
            DeclareLaunchArgument("concrete_block_x", default_value="6.0"),
            DeclareLaunchArgument("concrete_block_y", default_value="-2.5"),
            DeclareLaunchArgument("concrete_block_z", default_value="0.3"),
            DeclareLaunchArgument("concrete_block_yaw", default_value="0.0"),
            DeclareLaunchArgument("cbmp_execution_enabled", default_value="True"),
            DeclareLaunchArgument(
                "cbmp_default_trajectory_method",
                default_value="ACADOS_PATH_FOLLOWING",
            ),
            DeclareLaunchArgument("cbmp_execution_backend", default_value="topic"),
            DeclareLaunchArgument(
                "cbmp_execution_topic",
                default_value="/trajectory_controllers/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "cbmp_execution_action_name",
                default_value="/trajectory_controller_a2b/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "cbmp_execution_switch_controller", default_value="False"
            ),
            DeclareLaunchArgument(
                "cbmp_execution_activate_controller",
                default_value="trajectory_controllers",
            ),
            DeclareLaunchArgument(
                "enable_rviz_move_empty_interface", default_value="True"
            ),
            DeclareLaunchArgument(
                "rviz_move_empty_goal_topic", default_value="/goal_pose"
            ),
            DeclareLaunchArgument("rviz_move_empty_world_frame", default_value="world"),
            DeclareLaunchArgument(
                "rviz_move_empty_tool_frame", default_value="K8_tool_center_point"
            ),
            DeclareLaunchArgument(
                "rviz_move_empty_enable_topic", default_value="/cb_move_empty/enable"
            ),
            DeclareLaunchArgument("gazebo_master_port", default_value="11346"),
            DeclareLaunchArgument("cleanup_stale_gazebo", default_value="True"),
            DeclareLaunchArgument("use_perception", default_value="False"),
            DeclareLaunchArgument("concrete_rviz", default_value="True"),
            DeclareLaunchArgument("start_bt_action_server", default_value="True"),
            DeclareLaunchArgument("bt_start_delay_s", default_value="8.0"),
            DeclareLaunchArgument(
                "bt_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_behavior_tree"),
                        "config",
                        "default.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_behavior_tree"),
                        "rviz",
                        "concrete_bt.rviz",
                    ]
                ),
            ),
            gazebo_master_uri,
            cleanup_gazebo_processes,
            gazebo_after_cleanup,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                    )
                ),
                launch_arguments={
                    "world": world_path,
                    "pause": start_paused,
                    "gui": gui,
                }.items(),
                condition=UnlessCondition(cleanup_stale_gazebo),
            ),
            delayed_runtime_bringup,
            unpause_after_block_spawn,
            unpause_after_trajectory,
        ]
    )
