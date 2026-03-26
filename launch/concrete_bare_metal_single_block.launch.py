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
    auto_run = LaunchConfiguration("auto_run")
    auto_execute = LaunchConfiguration("auto_execute")
    auto_dry_run = LaunchConfiguration("auto_dry_run")
    startup_delay_s = LaunchConfiguration("startup_delay_s")
    seed_world_model = LaunchConfiguration("seed_world_model")
    approach_offset_m = LaunchConfiguration("approach_offset_m")

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
    world_model_seed_b0 = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_perception"),
            "config",
            "world_model_seed_b0.yaml",
        ]
    )
    world_model_seed_none = PathJoinSubstitution(
        [
            FindPackageShare("concrete_block_perception"),
            "config",
            "world_model_seed_none.yaml",
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
            "enable_livox_sim:=simple",
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
            DeclareLaunchArgument("seed_world_model", default_value="True"),
            DeclareLaunchArgument("auto_run", default_value="True"),
            DeclareLaunchArgument("auto_execute", default_value="True"),
            DeclareLaunchArgument("auto_dry_run", default_value="False"),
            DeclareLaunchArgument("startup_delay_s", default_value="10.0"),
            DeclareLaunchArgument("approach_offset_m", default_value="2.0"),
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
                        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                    )
                ),
                launch_arguments={
                    "world": PathJoinSubstitution(
                        [FindPackageShare("testsite_description"), "worlds", "empty.world"]
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
                        [FindPackageShare("concrete_block_perception"), "launch", "perception.launch.py"]
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
                        [FindPackageShare("concrete_block_perception"), "launch", "perception.launch.py"]
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
                        ["'", seed_world_model, "'.lower() != 'true'"]
                    )
                ),
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
                        name="spawner_trajectory_controllers",
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
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="concrete_block_motion_planning",
                        executable="motion_planning_node.py",
                        name="concrete_block_motion_planning_node",
                        output="screen",
                        parameters=[
                            motion_planning_config,
                            {
                                "planner.backend": "concrete",
                                "execution.enabled": True,
                                "execution.backend": "topic",
                                "execution.trajectory_topic": "/trajectory_controllers/commands",
                                "execution.joint_states_topic": "/joint_states",
                                "execution.motion_check_timeout_s": 0.0,
                                "named_configurations_file": named_cfg_file,
                                "wall_plan_file": wall_plan_file,
                                "geometric_optimized_params_file": optimized_params_file,
                            },
                        ],
                    )
                ],
            ),
            TimerAction(
                period=startup_delay_s,
                actions=[
                    Node(
                        package="concrete_block_motion_planning",
                        executable="bare_metal_single_block_client.py",
                        name="bare_metal_single_block_client",
                        output="screen",
                        parameters=[
                            {
                                "execute": auto_execute,
                                "dry_run": auto_dry_run,
                                "startup_delay_s": 0.0,
                                "approach_offset_m": approach_offset_m,
                            }
                        ],
                    )
                ],
                condition=IfCondition(auto_run),
            ),
        ]
    )
