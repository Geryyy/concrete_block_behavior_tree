from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    estimators_config = PathJoinSubstitution(
        [
            FindPackageShare("epsilon_crane_bringup_common"),
            "config",
            "estimators.yaml",
        ]
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="epsilon_crane_estimators",
                executable="pump_flow_estimator",
                output="both",
                parameters=[estimators_config, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_desired",
                arguments=[
                    "--frame-id",
                    "K0_mounting_base",
                    "--child-frame-id",
                    "desired/K0_mounting_base",
                ],
                output="screen",
            ),
        ]
    )
