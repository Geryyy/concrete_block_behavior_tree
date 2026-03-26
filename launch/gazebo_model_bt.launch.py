from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
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
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    logs_sim = LaunchConfiguration("logs_sim")
    zed2i_sim = LaunchConfiguration("zed2i_sim")
    ait_stereo_sim = LaunchConfiguration("ait_stereo_sim")
    enable_livox_sim = LaunchConfiguration("enable_livox_sim")
    initial_pose = LaunchConfiguration("initial_pose")
    log_on_truck = LaunchConfiguration("log_on_truck")
    parallel_pile = LaunchConfiguration("parallel_pile")
    parallel_logs = LaunchConfiguration("parallel_logs")
    cross_pile = LaunchConfiguration("cross_pile")
    chaos_pile = LaunchConfiguration("chaos_pile")
    motion_backend = LaunchConfiguration("motion_backend")
    tool = LaunchConfiguration("tool")
    gui = LaunchConfiguration("gui")
    use_perception = LaunchConfiguration("use_perception")
    seed_world_model = LaunchConfiguration("seed_world_model")
    use_timber_backend = IfCondition(
        PythonExpression(["'", motion_backend, "' == 'timber'"])
    )
    use_concrete_backend = IfCondition(
        PythonExpression(["'", motion_backend, "' == 'concrete'"])
    )

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_description"),
            "urdf",
            "timber_loader_AIT.urdf.xacro",
        ]
    )
    motion_planning_config = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_motion_planning"),
            "config",
            "motion_planning.yaml",
        ]
    )
    named_cfg_file = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_motion_planning"),
            "config",
            "named_configurations.yaml",
        ]
    )
    wall_plan_file = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_motion_planning"),
            "motion_planning",
            "data",
            "wall_plans.yaml",
        ]
    )
    optimized_params_file = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_motion_planning"),
            "motion_planning",
            "data",
            "optimized_params.yaml",
        ]
    )
    bt_common_config = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "bt_common.yaml"]
    )
    bt_operator_config = PathJoinSubstitution(
        [FindPackageShare("concrete_block_behavior_tree"), "config", "bt_operator.yaml"]
    )
    bt_move_empty_profile = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "profiles",
            "move_empty.yaml",
        ]
    )
    pzs100_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_behavior_tree"),
            "config",
            "ros2_control",
            "pzs100_trajectory_forward.ros2_control.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_bringup_hmi"),
            "rviz",
            "crane_data.rviz",
        ]
    )
    world_model_seed_none = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_perception"),
            "config",
            "world_model_seed_none.yaml",
        ]
    )
    world_model_seed_b0 = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_perception"),
            "config",
            "world_model_seed_b0.yaml",
        ]
    )

    use_sim_time_param = {"use_sim_time": use_sim_time}
    robot_description_gazebo = Command(
        [
            "xacro",
            " ",
            xacro_path,
            " ",
            "gazebo:=true",
            " ",
            "rigid:=true ",
            "sim_hydraulics:=true",
            " ",
            "enable_logs_sim:=",
            logs_sim,
            " ",
            "enable_zed2i_sim:=",
            zed2i_sim,
            " ",
            "enable_ait_stereo_sim:=",
            ait_stereo_sim,
            " ",
            "enable_livox_sim:=",
            enable_livox_sim,
            " ",
            "initial_pose:=",
            initial_pose,
            " ",
            "tool:=",
            tool,
            " ",
            "post_setup:=134 ",
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
            "rigid:=true ",
            "sim_hydraulics:=false",
            " ",
            "initial_pose:=",
            initial_pose,
            " ",
            "tool:=",
            tool,
            " ",
            "post_setup:=134 ",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="logs_sim",
                default_value="False",
                description="publish log poses from gazebo simulation",
            ),
            DeclareLaunchArgument(
                name="zed2i_sim",
                default_value="False",
                description="Simulate zed2i camera within gazebo simulation",
            ),
            DeclareLaunchArgument(
                name="ait_stereo_sim",
                default_value="False",
                description="Simulate ait stereo camera within gazebo simulation",
            ),
            DeclareLaunchArgument(
                name="enable_livox_sim",
                default_value="simple",
                description="Simulate livox lidar within gazebo simulation ('', 'simple' or 'livox')",
            ),
            DeclareLaunchArgument(
                name="initial_pose",
                default_value="2",
                description="Define initial pose of crane, see readme for further details",
            ),
            DeclareLaunchArgument(
                name="plot",
                default_value="False",
                description="Compatibility flag; currently unused in concrete minimal bringup",
            ),
            DeclareLaunchArgument(
                name="tool",
                default_value="pzs100_description",
                description="Name of the package where the tool is defined",
            ),
            DeclareLaunchArgument(
                name="log_on_truck",
                default_value="True",
                description="Flag to spawn a log on truck bed",
            ),
            DeclareLaunchArgument(
                name="parallel_pile",
                default_value="False",
                description="Flag to spawn a pile of wood logs arranged in parallel",
            ),
            DeclareLaunchArgument(
                name="parallel_logs",
                default_value="False",
                description="Flag to spawn a set of wood logs arranged in parallel",
            ),
            DeclareLaunchArgument(
                name="cross_pile",
                default_value="False",
                description="Flag to spawn a pile of wood logs arranged orthogonally",
            ),
            DeclareLaunchArgument(
                name="chaos_pile",
                default_value="False",
                description="Flag to spawn a pile of wood logs arranged chaotically",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to launch gazebo and RViz GUI",
            ),
            DeclareLaunchArgument(
                name="motion_backend",
                default_value="timber",
                description="Active motion backend: timber or concrete",
            ),
            DeclareLaunchArgument(
                name="use_perception",
                default_value="true",
                description="Start concrete perception/world-model services for scan BTs",
            ),
            DeclareLaunchArgument(
                name="seed_world_model",
                default_value="true",
                description="Prime world model with a static B0 anchor block and skip perception processing stack",
            ),
            Node(
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
            ),
            Node(
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
            ),
            Node(
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
                condition=use_concrete_backend,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epsilon_crane_bringup_sim"),
                            "launch",
                            "gazebo_base.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "gui": gui,
                    "log_on_truck": log_on_truck,
                    "parallel_pile": parallel_pile,
                    "parallel_logs": parallel_logs,
                    "cross_pile": cross_pile,
                    "chaos_pile": chaos_pile,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("concrete_block_behavior_tree"),
                            "launch",
                            "concrete_estimators.launch.py",
                        ]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=use_concrete_backend,
            ),
            IncludeLaunchDescription(
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
                    "pipeline_mode": "idle",
                    "start_processing_stack": "false",
                    "world_model_overlay_params_file": world_model_seed_b0,
                }.items(),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_perception,
                            "'.lower() == 'true' and '",
                            seed_world_model,
                            "'.lower() == 'true'",
                        ]
                    )
                ),
            ),
            IncludeLaunchDescription(
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
                    "world_model_overlay_params_file": world_model_seed_none,
                }.items(),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_perception,
                            "'.lower() == 'true' and '",
                            seed_world_model,
                            "'.lower() != 'true'",
                        ]
                    )
                ),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
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
                            "start_bt_action_server": "True",
                            "bt_common_params_file": bt_common_config,
                            "bt_mode_params_file": bt_operator_config,
                            "bt_profile_params_file": bt_move_empty_profile,
                            "keyboard_node": "True",
                            "gui": gui,
                        }.items(),
                    )
                ],
                condition=use_timber_backend,
            ),
            IncludeLaunchDescription(
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
                    "start_bt_action_server": "True",
                    "bt_common_params_file": bt_common_config,
                    "bt_mode_params_file": bt_operator_config,
                    "bt_profile_params_file": bt_move_empty_profile,
                    "keyboard_node": "True",
                    "gui": gui,
                }.items(),
                condition=use_concrete_backend,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epsilon_crane_bringup_common"),
                            "launch",
                            "common_controllers.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "tool": tool,
                }.items(),
                condition=use_concrete_backend,
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-p",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epsilon_crane_bringup_common"),
                            "config",
                            "ros2_control",
                            "crane_controller_common.ros2_control.yaml",
                        ]
                    ),
                ],
                output="screen",
                condition=use_timber_backend,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("concrete_block_behavior_tree"),
                            "launch",
                            "timber_backend_compat.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "logs_sim": logs_sim,
                    "zed2i_sim": zed2i_sim,
                    "ait_stereo_sim": ait_stereo_sim,
                    "enable_livox_sim": enable_livox_sim,
                    "initial_pose": initial_pose,
                }.items(),
                condition=use_timber_backend,
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="timber_follow_joint_trajectory_proxy.py",
                name="timber_follow_joint_trajectory_proxy_concrete",
                output="screen",
                condition=use_concrete_backend,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epsilon_crane_bringup_common"),
                            "launch",
                            "loadbed_frame.launch.py",
                        ]
                    )
                ),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
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
                        condition=use_concrete_backend,
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
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
                        condition=use_timber_backend,
                    )
                ],
            ),
            IncludeLaunchDescription(
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
                    "motion_planning_params_file": motion_planning_config,
                    "default_trajectory_method": "FIXED_TIME_INTERPOLATION",
                    "named_configurations_file": named_cfg_file,
                    "wall_plan_file": wall_plan_file,
                    "geometric_optimized_params_file": optimized_params_file,
                    "execution_enabled": "true",
                    "execution_backend": "action",
                    "execution_action_name": "/trajectory_controller_a2b/follow_joint_trajectory",
                    "execution_switch_controller": "false",
                    "execution_activate_controller": "trajectory_controllers",
                    "planner_backend": "concrete",
                }.items(),
                condition=use_concrete_backend,
            ),
            IncludeLaunchDescription(
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
                    "motion_planning_params_file": motion_planning_config,
                    "named_configurations_file": named_cfg_file,
                    "wall_plan_file": wall_plan_file,
                    "geometric_optimized_params_file": optimized_params_file,
                    "execution_enabled": "true",
                    "execution_backend": "action",
                    "execution_action_name": "/trajectory_controller_a2b/follow_joint_trajectory",
                    "execution_switch_controller": "false",
                    "execution_activate_controller": "trajectory_controllers",
                    "planner_backend": "timber",
                    "planner_timber_a2b_service": "a2b_movement",
                    "planner_timber_goal_frame": "K0_mounting_base",
                    "planner_timber_move_empty_target_z": "2.36",
                }.items(),
                condition=use_timber_backend,
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_PACKAGE",
                value="concrete_block_behavior_tree",
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_BT_CATALOG",
                value=PathJoinSubstitution(
                    [
                        FindPackageShare("concrete_block_behavior_tree"),
                        "config",
                        "bt_panel_catalog.yaml",
                    ]
                ),
            ),
            SetEnvironmentVariable(
                name="BEHAVIOR_TREE_PANEL_MOVE_EMPTY_BT",
                value="/behavior_trees/move_empty.xml",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                parameters=[use_sim_time_param],
                arguments=["-d", rviz_config],
                condition=IfCondition(gui),
                output="screen",
            ),
        ]
    )
