from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    plot_controller_state = LaunchConfiguration("plot_controller_state")

    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    reference_generator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "task_space_reference_generator",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    layout_path_plotjuggler = PathJoinSubstitution(
        [
            FindPackageShare("ur10_admittance_controller_demo"),
            "config",
            "admittance_controller_state_plots.xml",
        ]
    )

    plotjuggler_starter = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path_plotjuggler],
        condition=IfCondition(plot_controller_state),
    )

    delay_reference_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[reference_generator_spawner],
        )
    )

    return [
        admittance_controller_spawner,
        delay_reference_after_controller_spawner,
        plotjuggler_starter,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "plot_controller_state",
            default_value="true",
            description="Enables the launch of the PlotJuggler tool for real-time visualization of F/T plots.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
