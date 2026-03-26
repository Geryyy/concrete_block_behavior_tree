from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    tool = LaunchConfiguration("tool")
    initial_pose = LaunchConfiguration("initial_pose")
    start_motion_planning = LaunchConfiguration("start_motion_planning")
    start_world_model = LaunchConfiguration("start_world_model")
    seed_world_model = LaunchConfiguration("seed_world_model")

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_description"),
            "urdf",
            "timber_loader_AIT.urdf.xacro",
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
    common_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_bringup_common"),
            "config",
            "ros2_control",
            "crane_controller_common.ros2_control.yaml",
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
            "enable_logs_sim:=False",
            " ",
            "enable_zed2i_sim:=False",
            " ",
            "enable_ait_stereo_sim:=False",
            " ",
            "enable_livox_sim:=",
            "simple",
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
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("gui", default_value="False"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            DeclareLaunchArgument("initial_pose", default_value="2"),
            DeclareLaunchArgument("start_motion_planning", default_value="True"),
            DeclareLaunchArgument("start_world_model", default_value="False"),
            DeclareLaunchArgument("seed_world_model", default_value="False"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description_gazebo},
                    {"use_sim_time": use_sim_time},
                    {"publish_frequency": 100.0},
                ],
                remappings=[
                    ("robot_description", "robot_description_full"),
                    ("tf", "tf_gazebo"),
                    ("tf_static", "tf_static_gazebo"),
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("gazebo_ros"),
                            "launch",
                            "gazebo.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "world": PathJoinSubstitution(
                        [
                            FindPackageShare("testsite_description"),
                            "worlds",
                            "empty.world",
                        ]
                    ),
                    "pause": "False",
                    "gui": gui,
                }.items(),
            ),
            Node(
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
                condition=IfCondition(seed_world_model),
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
                    "world_model_overlay_params_file": world_model_seed_none,
                }.items(),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_world_model,
                            "'.lower() == 'true' and '",
                            seed_world_model,
                            "'.lower() != 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="timber_follow_joint_trajectory_proxy.py",
                name="timber_follow_joint_trajectory_proxy_concrete",
                output="screen",
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "--controller-manager-timeout",
                            "60",
                            "-p",
                            common_controller_params,
                            "joint_state_broadcaster",
                        ],
                        output="screen",
                    )
                ],
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
                    "default_trajectory_method": "TOPPRA_PATH_FOLLOWING",
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
                condition=IfCondition(start_motion_planning),
            ),
        ]
    )
