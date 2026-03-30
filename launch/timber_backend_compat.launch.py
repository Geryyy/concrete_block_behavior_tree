from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    logs_sim = LaunchConfiguration("logs_sim")
    zed2i_sim = LaunchConfiguration("zed2i_sim")
    ait_stereo_sim = LaunchConfiguration("ait_stereo_sim")
    enable_livox_sim = LaunchConfiguration("enable_livox_sim")
    initial_pose = LaunchConfiguration("initial_pose")

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_description"),
            "urdf",
            "timber_loader_AIT.urdf.xacro",
        ]
    )
    a2b_param_path = PathJoinSubstitution(
        [FindPackageShare("epsilon_crane_bringup_mp"), "config", "motion_planning", "a2b_parameter.yaml"]
    )
    mp_param_path = PathJoinSubstitution(
        [FindPackageShare("epsilon_crane_bringup_mp"), "config", "motion_planning", "mp_parameter_pzs100.yaml"]
    )
    grip_traj_param_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_bringup_mp"),
            "config",
            "motion_planning",
            "grip_traj_parameter_pzs100.yaml",
        ]
    )
    collision_objects_path = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_bringup_mp"),
            "config",
            "motion_planning",
            "collision_objects.yaml",
        ]
    )

    compat_robot_description = Command(
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
            "tool:=pzs100_description",
            " ",
            "post_setup:=134 ",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("logs_sim", default_value="False"),
            DeclareLaunchArgument("zed2i_sim", default_value="False"),
            DeclareLaunchArgument("ait_stereo_sim", default_value="False"),
            DeclareLaunchArgument("enable_livox_sim", default_value=""),
            DeclareLaunchArgument("initial_pose", default_value="1"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher_timber_compat",
                parameters=[
                    {"robot_description": compat_robot_description},
                    {"use_sim_time": use_sim_time},
                    {"publish_frequency": 0.01},
                ],
                remappings=[
                    ("robot_description", "robot_description_full_timber_compat"),
                    ("tf", "tf_timber_compat"),
                    ("tf_static", "tf_static_timber_compat"),
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
                output="log",
            ),
            Node(
                package="crane_tools_description",
                executable="crane_tools_description_publisher",
                name="crane_tools_description_publisher_timber_compat",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pzs100_description"),
                            "config",
                            "gripper_parameter.yaml",
                        ]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("crane_tools_description", "/crane_tools_description_timber_compat")],
                output="log",
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="dummy_joint_state_publisher.py",
                name="dummy_joint_state_publisher",
                output="log",
                parameters=[{"topic": "/joint_states"}, {"publish_rate_hz": 10.0}],
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="gripper_state_bridge.py",
                name="gripper_state_bridge",
                parameters=[{"state_topic": "/gripper_state"}, {"publish_rate_hz": 5.0}],
                output="log",
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="joint_state_timber_compat.py",
                name="joint_state_timber_compat",
                output="log",
                parameters=[{"source_topic": "/joint_states"}, {"target_topic": "/joint_states_timber_compat"}],
            ),
            Node(
                package="concrete_block_motion_planning",
                executable="timber_follow_joint_trajectory_proxy.py",
                name="timber_follow_joint_trajectory_proxy",
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_outer_jaw_alias",
                arguments=[
                    "--frame-id",
                    "K10_left_rail",
                    "--child-frame-id",
                    "K10_outer_jaw",
                ],
                output="log",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_inner_jaw_alias",
                arguments=[
                    "--frame-id",
                    "K12_right_rail",
                    "--child-frame-id",
                    "K12_inner_jaw",
                ],
                output="log",
            ),
            Node(
                package="timber_crane_motion_planning",
                executable="grip_traj_server",
                parameters=[
                    mp_param_path,
                    grip_traj_param_path,
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description_topic": "robot_description_full_timber_compat",
                    },
                ],
                remappings=[("joint_states", "/joint_states_timber_compat")],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "INFO",
                    "-r",
                    "crane_tools_description:=/crane_tools_description_timber_compat",
                ],
                output="screen",
            ),
            Node(
                package="timber_crane_motion_planning",
                executable="a2b_ilqr_jerk_server",
                parameters=[
                    mp_param_path,
                    a2b_param_path,
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description_topic": "robot_description_full_timber_compat",
                        "timeout": 120.0,
                        "a2bOptions": {
                            "compensateElasticity": True,
                            "max_iter": 3,
                            "numThreads": 2,
                            "tend_min": 4.0,
                            "t_margin": 3.0,
                            "use_reduction": 1,
                        },
                    },
                ],
                remappings=[("joint_states", "/joint_states_timber_compat")],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "INFO",
                    "-r",
                    "crane_tools_description:=/crane_tools_description_timber_compat",
                ],
                output="screen",
            ),
            Node(
                package="collision_body_handler",
                executable="collision_body_handler",
                output="log",
                parameters=[
                    {"collision_objects_file": collision_objects_path},
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
