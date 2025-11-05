from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import OpaqueFunction


def setup_launch(context, *args, **kwargs):
    ROBOT_NAME = "fer"

    # Get the tf_prefix value from the LaunchConfiguration and context
    description_xacro_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
        "hand": LaunchConfiguration("hand").perform(context),
    }

    description_semantic_xacro_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
        "hand": LaunchConfiguration("hand").perform(context),
    }

    # Build the MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name="acg_resources_fer_moveit_config")
        .robot_description(mappings=description_xacro_args)
        .robot_description_semantic(mappings=description_semantic_xacro_args)
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(generate_demo_launch(moveit_config))

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
        description="Whether to use the robot's hand or not.",
    )

    declared_arguments.append(hand_arg)
    declared_arguments.append(tf_prefix_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=setup_launch)]
    )
