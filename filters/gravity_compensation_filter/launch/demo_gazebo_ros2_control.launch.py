from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    gazebo_ros2_control_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_ur10_moveit_config"),
                    "launch",
                    "gazebo_ros2_control_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "safety_limits": "true",
            "runtime_config_package": "gravity_compensation_filter",
            "controllers_file": "test/config/ur10_gravity_compensation_test_controllers.yaml",
            "tf_prefix": "",
            "launch_rviz": "false",
            "enable_ft_sensing": "true",
            "enable_effort_interfaces": "false",
            "ft_plot": "false",
            "ee_config_package": "gravity_compensation_filter",
            "ee_config_file": "test/config/gravity_compensation_filter_test_ee_config.yaml",
        }.items(),
    )

    return [gazebo_ros2_control_demo_launch]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
