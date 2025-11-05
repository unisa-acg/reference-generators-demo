import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    ROBOT_NAME = "fer"

    # Value argument from launch configuration
    gui = LaunchConfiguration("gui")
    controller = LaunchConfiguration("controller")
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_state_publisher_frequency = LaunchConfiguration(
        "robot_state_publisher_frequency"
    )
    hand = LaunchConfiguration("hand")
    enable_effort_interfaces = LaunchConfiguration("enable_effort_interfaces")
    controllers_file = LaunchConfiguration("controllers_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_config_package = LaunchConfiguration("runtime_config_package")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file_path = LaunchConfiguration("gazebo_world_file_path")

    # Get controllers file path
    controllers_file = controllers_file.perform(context)
    controllers_file = PathJoinSubstitution(["config", controllers_file])
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), controllers_file]
    )

    # Get URDF via xacro
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fer_moveit_config"),
            "config",
            "fer.urdf.xacro",
        ]
    )

    # Load robot description without mock hardware
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "use_mock_hardware:=false",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "gazebo:=",
            "true",
            " ",
            "hand:=",
            hand,
            " ",
            "enable_effort_interfaces:=",
            enable_effort_interfaces,
            " ",
            "parameter_file:=",
            controllers_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_state_publisher_frequency = {
        "publish_frequency": LaunchConfiguration("robot_state_publisher_frequency")
    }

    # Prepare Gazebo resources and launch path:
    # this is needed by Gazebo to find the meshes referenced by the URDF
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        get_package_share_directory("franka_description") + "/.."
    )

    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    # Include Gazebo launch file to launch its environment
    launches = []

    if gazebo_world_file_path.perform(context) == "":
        gazebo_world = "empty.sdf"
    else:
        gazebo_world = PathJoinSubstitution(
            [FindPackageShare(gazebo_world_package), gazebo_world_file_path]
        ).perform(context)

    launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments=[("gz_args", [" -r -v 4 " + gazebo_world])],
        )
    )

    # Define all nodes that have to be launched

    # Spawn the model in Gazebo. This also loads the plugin and starts the control node.
    # At the time of writing this script, the 'create' command does not allow spawning
    # the robot at a specific joint configuration
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            ROBOT_NAME,
            "-allow_renaming",
            "true",
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_publisher_frequency],
    )

    # Load RViz configuration

    rviz_config_file = rviz_config_file.perform(context)
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package), "config", rviz_config_file]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
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

    motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
    )

    # Delay `joint_state_broadcaster` start after spawning the model in Gazebo
    delay_controllers_after_gz_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, motion_controller_spawner],
        )
    )

    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # This file contains the parameters for the bridge between ROS2 and
    # Ignition Gazebo
    gz_params_bridge = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fer_moveit_config"),
            "config",
            "gz_params_bridge.yaml",
        ]
    ).perform(context)

    gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={gz_params_bridge}",
        ],
        output="screen",
    )

    nodes = [
        gz_spawn_entity,
        robot_state_pub_node,
        delay_controllers_after_gz_spawn,
        delay_rviz_after_joint_state_broadcaster_spawner,
        gazebo_ros_bridge_cmd,
    ]

    return nodes + launches


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
            "controller",
            default_value="fer_forward_position_controller",
            description="Load the forward position controller by default.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_effort_interfaces",
            default_value="false",
            description="Enable effort interfaces for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the parameters for the controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="acg_resources_fer_moveit_config",
            description="Package with the RViz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="ros2_control_demo.rviz",
            description="RViz config file to use.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="acg_resources_fer_moveit_config",
            description="Package with the controller's configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_package",
            default_value="acg_resources_fer_moveit_config",
            description="Package with the Gazebo world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file_path",
            default_value="",
            description="The path to the Gazebo world file inside the "
            "`gazebo_world_package`. If not specified, the default empty world is used.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
