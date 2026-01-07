from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch
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
    ld.add_action(generate_setup_assistant_launch(moveit_config))

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
