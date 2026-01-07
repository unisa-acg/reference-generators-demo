from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def setup_launch(context, *args, **kwargs):
    ROBOT_NAME = "fer"

    # Get the tf_prefix value from the LaunchConfiguration and context
    xacro_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
        "hand": LaunchConfiguration("hand").perform(context),
    }

    # Build the MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name="xxx_resources_fer_moveit_config")
        .robot_description(mappings=xacro_args)
        .robot_description_semantic(mappings=xacro_args)
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(generate_moveit_rviz_launch(moveit_config))

    return ld.entities


def generate_launch_description():
    declared_arguments = []

    # Declare the tf_prefix argument
    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix", default_value="fer_", description="Prefix for the tf names."
    )
    hand_arg = DeclareLaunchArgument(
        "hand",
        default_value="true",
        description="Flag to enable the hand of the robot.",
    )

    declared_arguments.append(tf_prefix_arg)
    declared_arguments.append(hand_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=setup_launch)]
    )


# This function is redefined to receive relative paths for RViz config files


def generate_moveit_rviz_launch(moveit_config):
    """
    Launch file for rviz
    """

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_config_path = PathJoinSubstitution(
        [
            str(moveit_config.package_path),
            "config",
            rviz_config,
        ]
    )

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", rviz_config_path],
        parameters=rviz_parameters,
    )

    return ld
