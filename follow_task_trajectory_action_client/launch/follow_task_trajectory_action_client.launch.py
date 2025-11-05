from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the launch arguments to accept parameters from the command line
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        description="Name of the action",
    )
    input_trajectory_filename_arg = DeclareLaunchArgument(
        "input_trajectory_filename",
        description="Name of the file containing the input trajectory",
    )
    fraction_feedback_messages_to_save_arg = DeclareLaunchArgument(
        "fraction_feedback_messages_to_save",
        default_value="25",
        description="Define the portion of feedback messages to be saved. If its value "
        "is N, 1/N of total number of messages will be output to bag file.",
    )
    output_path_arg = DeclareLaunchArgument(
        "output_path",
        default_value="",
        description="Path to the output bag file",
    )

    # Define the LaunchConfiguration variables
    action_name = LaunchConfiguration("action_name")
    input_trajectory_filename = LaunchConfiguration("input_trajectory_filename")
    fraction_feedback_messages_to_save = LaunchConfiguration(
        "fraction_feedback_messages_to_save"
    )
    output_path = LaunchConfiguration("output_path")

    action_client = Node(
        package="follow_task_trajectory_action_client",
        executable="follow_task_trajectory_action_client",
        parameters=[
            {
                "action_name": action_name,
                "fraction_feedback_messages_to_save": fraction_feedback_messages_to_save,
                "input_trajectory_filename": input_trajectory_filename,
                "output_path": output_path,
            }
        ],
    )

    return LaunchDescription(
        [
            action_name_arg,
            input_trajectory_filename_arg,
            fraction_feedback_messages_to_save_arg,
            output_path_arg,
            action_client,
        ]
    )
