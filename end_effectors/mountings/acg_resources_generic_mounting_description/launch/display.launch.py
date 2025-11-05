from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_path = FindPackageShare("acg_resources_generic_mounting_description")
    urdf_path = PathJoinSubstitution(
        [package_path, "urdf", "generic_mounting_visualization.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution([package_path, "config", "display.rviz"])

    mounting_height = DeclareLaunchArgument(
        "height",
        default_value="0.05",
        description="Height of the mounting",
    )
    mounting_radius = DeclareLaunchArgument(
        "radius",
        default_value="0.1",
        description="Radius of the mounting",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " height:=",
                LaunchConfiguration("height"),
                " radius:=",
                LaunchConfiguration("radius"),
            ]
        ),
        value_type=str,
    )

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
            mounting_height,
            mounting_radius,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
