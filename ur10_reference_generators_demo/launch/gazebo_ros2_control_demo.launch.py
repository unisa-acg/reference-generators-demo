# Based on Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/launch/ur_sim_control.launch.py

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
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


DEFAULT_CONTROLLERS_FILE = "ur10_cpc_simulation.yaml"


def launch_setup(context, *args, **kwargs):
    # UR10 specific arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    initial_positions_package = LaunchConfiguration("initial_positions_package")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_state_publisher_frequency_arg = LaunchConfiguration(
        "robot_state_publisher_frequency"
    )
    # Spawn position in Gazebo
    spawn_z = LaunchConfiguration("spawn_z")

    # If a controller file is not specified by the user, choose which one to use among default ones
    controllers_file = controllers_file.perform(context)

    # If the controllers_file is a relative path, it is converted to an absolute path
    if len(Path(controllers_file).parents) == 1:
        # If the controllers_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        controllers_file = PathJoinSubstitution(["config", controllers_file])

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), controllers_file]
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )

    gazebo_world = "empty.sdf"

    initial_positions_path = PathJoinSubstitution(
        [FindPackageShare(initial_positions_package), "config", initial_positions_file]
    )
    # We create our own robot_description_content instead of using the one provided
    # in ur_simulation_gz/launch/ur_sim_control.launch.py, as the latter lacks the following parameters:
    # - enable_ft_sensing;
    # that need to be passed to the xacro file for launching the simulation.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "ur_type:=ur10",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur10",
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions_path,
            " ",
            "generate_ros2_control_tags:=true",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_frequency = {
        "publish_frequency": robot_state_publisher_frequency_arg
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_state_publisher_frequency,
        ],
    )

    # We create our own rviz_node because the one provided in
    # ur_simulation_gz/launch/ur_sim_control.launch.py does not allow for
    # changing the rviz_config_file, and we want to use our own rviz configuration file.
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package), rviz_config_file]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(launch_rviz),
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

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # GZ nodes
    # As a consequence of what wrote above for the robot_description, we create our gz_spawn_entity node
    # because the one provided in the launch file ur_simulation_gz/launch/ur_sim_control.launch.py only
    # accepts their robot_description_content
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
            "-z",
            spawn_z,
        ],
    )

    gz_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    conditional_gz_launch_description = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_file),
            launch_arguments={"gz_args": " -r -v 4 " + gazebo_world}.items(),
        )
    ]

    gz_launch_description = LaunchDescription(conditional_gz_launch_description)

    # This file contains the parameters for the bridge between ROS2 and
    # Ignition Gazebo without putting a condition as done for the torque
    gz_params_bridge = PathJoinSubstitution(
        [
            FindPackageShare("ur10_reference_generators_demo"),
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

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gz_spawn_entity,
        gz_launch_description,
        gazebo_ros_bridge_cmd,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # UR10 specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_effort_interfaces",
            default_value="false",
            description="If true enables the effort command interfaces.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur10_reference_generators_demo",
            description="Package with the controller's configuration. \
            Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=DEFAULT_CONTROLLERS_FILE,
            description="YAML file with the controllers configuration. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="ur10_reference_generators_demo",
            description="Package with the RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="config/view_robot.rviz",
            description="RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="cartesian_pose_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_package",
            default_value="ur_description",
            description="Package with the initial positions of the robot joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="File with the initial positions of the robot joints.",
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
            "spawn_z",
            default_value="0.0",
            description="Z coordinate for the robot spawn position in Gazebo.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
