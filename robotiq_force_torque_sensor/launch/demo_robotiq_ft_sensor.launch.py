from launch import LaunchDescription

from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robotiq_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robotiq_ft_sensor_hardware"),
                    "launch",
                    "robotiq_hardware.launch.py",
                ]
            )
        )
    )

    plotjuggler_layout = PathJoinSubstitution(
        [
            FindPackageShare("robotiq_force_torque_sensor"),
            "config",
            "robotiq_force_torque_measurements_plot.xml",
        ]
    )

    plotjuggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", plotjuggler_layout],
    )

    nodes_to_start = [
        robotiq_hardware,
        plotjuggler,
    ]

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
