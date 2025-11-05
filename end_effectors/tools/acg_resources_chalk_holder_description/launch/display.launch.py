from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_path = FindPackageShare("acg_resources_chalk_holder_description")
    urdf_path = PathJoinSubstitution(
        [package_path, "urdf", "chalk_holder_visualization.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution([package_path, "config", "display.rviz"])

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
        ]
    )
