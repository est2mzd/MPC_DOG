from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    config_path = LaunchConfiguration("config_path")
    log_path = LaunchConfiguration("log_path")
    driver_params_path = LaunchConfiguration("driver_params_path")
    automatic_recovery = LaunchConfiguration("automatic_recovery")

    replacement_driver = Node(
        package="robot_driver",
        executable="robot_driver_node",
        name="robot_driver",
        namespace=namespace,
        output="screen",
        parameters=[driver_params_path],
        remappings=[
            ("control/joint_command", "control/walking_joint_command"),
        ],
    )

    recovery = Node(
        package="recovery_controller",
        executable="recovery_controller_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "config_path": config_path,
                "log_path": log_path,
                "use_sim_time": True,
                "automatic_recovery": ParameterValue(
                    automatic_recovery, value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_1"),
            DeclareLaunchArgument("robot_type", default_value="go2"),
            DeclareLaunchArgument("driver_params_path"),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("recovery_controller"),
                        "config",
                        "go2_recovery.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("log_path", default_value=""),
            DeclareLaunchArgument("automatic_recovery", default_value="true"),
            replacement_driver,
            recovery,
        ]
    )
