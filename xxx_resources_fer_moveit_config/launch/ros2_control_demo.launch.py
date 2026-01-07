from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix", default_value="fer_", description="Prefix for the tf names."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_state_publisher_frequency",
            default_value="1000.0",
            description="The frequency (in Hz) at which the robot_state_publisher publishes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="true",
            description="Flag to enable the hand of the robot.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("xxx_resources_fer_moveit_config"),
                    "config",
                    "fer.urdf.xacro",
                ]
            ),
            " ",
            "tf_prefix:=",
            LaunchConfiguration("tf_prefix"),
            " ",
            "hand:=",
            LaunchConfiguration("hand"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_state_publisher_frequency = {
        "publish_frequency": LaunchConfiguration("robot_state_publisher_frequency")
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("xxx_resources_fer_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("xxx_resources_fer_moveit_config"),
            "config",
            "ros2_control_demo.rviz",
        ]
    )

    # Define all nodes that have to be launched

    # Start an independent control node with controller manager
    # This node will be instantiated with the name '/controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_publisher_frequency],
    )

    # Start RViz without loading plugins associated to planning
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fer_forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
