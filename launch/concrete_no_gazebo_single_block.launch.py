from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    tool = LaunchConfiguration("tool")
    initial_pose = LaunchConfiguration("initial_pose")
    seed_world_model = LaunchConfiguration("seed_world_model")
    auto_run = LaunchConfiguration("auto_run")
    startup_delay_s = LaunchConfiguration("startup_delay_s")
    approach_offset_m = LaunchConfiguration("approach_offset_m")
    start_pose_xyz = LaunchConfiguration("start_pose_xyz")
    start_pose_yaw_deg = LaunchConfiguration("start_pose_yaw_deg")

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

    robot_description = Command(
        [
            "xacro",
            " ",
            xacro_path,
            " ",
            "gazebo:=false",
            " ",
            "rigid:=true ",
            "fake_hardware:=false ",
            "real_hardware:=false ",
            "sim_hydraulics:=false",
            " ",
            "load_ros2_control:=false",
            " ",
            "enable_logs_sim:=False",
            " ",
            "enable_zed2i_sim:=False",
            " ",
            "enable_ait_stereo_sim:=False",
            " ",
            "enable_livox_sim:=false",
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
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("tool", default_value="pzs100_description"),
            DeclareLaunchArgument("initial_pose", default_value="2"),
            DeclareLaunchArgument("seed_world_model", default_value="True"),
            DeclareLaunchArgument("auto_run", default_value="True"),
            DeclareLaunchArgument("startup_delay_s", default_value="3.0"),
            DeclareLaunchArgument("approach_offset_m", default_value="2.0"),
            DeclareLaunchArgument("start_pose_xyz", default_value="[-10.98, -3.71, 2.15]"),
            DeclareLaunchArgument("start_pose_yaw_deg", default_value="-180.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": use_sim_time},
                    {"publish_frequency": 30.0},
                ],
                remappings=[("robot_description", "robot_description_full")],
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
                    PythonExpression(["'", seed_world_model, "'.lower() not in ('true', '1', 'yes')"])
                ),
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="motion_planning_node.py",
                name="concrete_block_motion_planning_node",
                output="screen",
                parameters=[
                    motion_planning_config,
                    {
                        "planner.backend": "concrete",
                        "execution.enabled": False,
                        "named_configurations_file": named_cfg_file,
                        "wall_plan_file": wall_plan_file,
                        "geometric_optimized_params_file": optimized_params_file,
                        "use_sim_time": use_sim_time,
                    },
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
                                "execute": False,
                                "dry_run": True,
                                "startup_delay_s": 0.0,
                                "use_tf_start_pose": False,
                                "approach_offset_m": approach_offset_m,
                                "start_pose_xyz": start_pose_xyz,
                                "start_pose_yaw_deg": start_pose_yaw_deg,
                            }
                        ],
                    )
                ],
                condition=IfCondition(auto_run),
            ),
        ]
    )
