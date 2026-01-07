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


DEFAULT_CONTROLLERS_FILE = "ur10_simulation_controllers.yaml"
DEFAULT_EFFORT_CONTROLLERS_FILE = "ur10_simulation_effort_controllers.yaml"
DEFAULT_FT_CONTROLLERS_FILE = "ur10_simulation_ft_controllers.yaml"
DEFAULT_EFFORT_FT_CONTROLLERS_FILE = "ur10_simulation_effort_ft_controllers.yaml"


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
    enable_effort_interfaces = LaunchConfiguration("enable_effort_interfaces")
    enable_ft_sensing = LaunchConfiguration("enable_ft_sensing")
    ft_plot = LaunchConfiguration("ft_plot")
    # End-effector arguments
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")
    robot_state_publisher_frequency_arg = LaunchConfiguration(
        "robot_state_publisher_frequency"
    )
    # Spawn position in Gazebo
    spawn_z = LaunchConfiguration("spawn_z")
    # Gazebo world
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file_path = LaunchConfiguration("gazebo_world_file_path")

    # If a controller file is not specified by the user, choose which one to use among default ones
    controllers_file = controllers_file.perform(context)
    if controllers_file == DEFAULT_CONTROLLERS_FILE:
        if (
            enable_effort_interfaces.perform(context) == "true"
            and enable_ft_sensing.perform(context) == "true"
        ):
            controllers_file = DEFAULT_EFFORT_FT_CONTROLLERS_FILE
        else:
            if enable_effort_interfaces.perform(context) == "true":
                controllers_file = DEFAULT_EFFORT_CONTROLLERS_FILE

            if enable_ft_sensing.perform(context) == "true":
                controllers_file = DEFAULT_FT_CONTROLLERS_FILE

    # If the controllers_file is a relative path, it is converted to an absolute path
    if len(Path(controllers_file).parents) == 1:
        # If the controllers_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        controllers_file = PathJoinSubstitution(["config", controllers_file])

    # If the ee_config_file is a relative path, it is converted to an absolute path
    if len(Path(ee_config_file.perform(context)).parents) == 1:
        # If the ee_config_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        ee_config_file = PathJoinSubstitution(["config", ee_config_file])

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), controllers_file]
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )

    ee_config_file_path = PathJoinSubstitution(
        [FindPackageShare(ee_config_package), ee_config_file]
    ).perform(context)
    if not ee_config_file.perform(context):
        ee_config_file_path = ""

    # If the user does not specify a Gazebo world file, we use the default one
    if gazebo_world_file_path.perform(context) == "":
        # If the force/torque sensing is enabled, we use the world with the F/T sensor
        if enable_ft_sensing.perform(context).lower() == "true":
            gazebo_world = PathJoinSubstitution(
                [
                    FindPackageShare("xxx_resources_ft_sensor_gazebo_description"),
                    "world",
                    "world_with_ft_sensor.sdf",
                ]
            ).perform(context)
        # If the force/torque sensing is not enabled, we use the empty world
        else:
            gazebo_world = "empty.sdf"
    # If the user specifies a Gazebo world file, we use it regardless of the value of the `enable_ft_sensing` argument
    else:
        gazebo_world = PathJoinSubstitution(
            [FindPackageShare(gazebo_world_package), gazebo_world_file_path]
        ).perform(context)

    initial_positions_path = PathJoinSubstitution(
        [FindPackageShare(initial_positions_package), "config", initial_positions_file]
    )
    # We create our own robot_description_content instead of using the one provided
    # in ur_simulation_gz/launch/ur_sim_control.launch.py, as the latter lacks the following parameters:
    # - enable_ft_sensing;
    # - ee_config_file;
    # that need to be passed to the xacro file for launching the simulation.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
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
            "enable_effort_interfaces:=",
            enable_effort_interfaces,
            " ",
            "enable_ft_sensing:=",
            enable_ft_sensing,
            " ",
            "ee_config_file:=",
            ee_config_file_path,
            " ",
            "initial_positions_file:=",
            initial_positions_path,
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

    # A custom rviz_node is created because the one provided in
    # ur_simulation_gz/launch/ur_sim_control.launch.py does not allow for
    # changing the rviz_config_file, and a custom rviz configuration file is needed.
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

    force_torque_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
        condition=IfCondition(enable_ft_sensing),
    )

    layout_path_for_gazebo_ft_plot = PathJoinSubstitution(
        [
            FindPackageShare("gazebo_force_torque_sensor"),
            "config",
            "gazebo_force_torque_measurements_plot.xml",
        ]
    )

    plot_gazebo_ft_measurements = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path_for_gazebo_ft_plot],
        condition=IfCondition(enable_ft_sensing and ft_plot),
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

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/force_torque@geometry_msgs/msg/Wrench@ignition.msgs.Wrench"],
        output="screen",
        condition=IfCondition(enable_ft_sensing),
    )

    # This file contains the parameters for the bridge between ROS2 and
    # Ignition Gazebo without putting a condition as done for the torque
    gz_params_bridge = PathJoinSubstitution(
        [
            FindPackageShare("xxx_resources_ur10_moveit_config"),
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
        force_torque_sensor_broadcaster,
        gz_spawn_entity,
        gz_launch_description,
        bridge,
        plot_gazebo_ft_measurements,
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
            default_value="xxx_resources_ur10_moveit_config",
            description="Package with the controller's configuration. "
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=DEFAULT_CONTROLLERS_FILE,
            description="YAML file with the controllers configuration. "
            "The file is searched under the `config` folder of the specified "
            "`runtime_config_package` or in the provided relative path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="xxx_resources_ur10_description",
            description="Description package with robot URDF/XACRO files. "
            "Usually the argument is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur10_with_end_effector.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. "
            "If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="xxx_resources_ur10_description",
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
            default_value="joint_trajectory_controller",
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
            "enable_ft_sensing",
            default_value="false",
            description="Enable force/torque sensing.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ft_plot",
            default_value="false",
            description="Enables the launch of the PlotJuggler tool for real-time "
            "visualization of F/T plots.",
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
            description="The frequency (in Hz) at which the robot_state_publisher "
            "publishes.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_package",
            default_value="xxx_resources_ur10_moveit_config",
            description="Package with the end-effector configuration file under the "
            "`config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_file",
            default_value="",
            description="End-effector configuration file. "
            "The file is searched under the `config` folder of the specified "
            "`ee_config_package` or in the provided relative path.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z coordinate for the robot spawn position in Gazebo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_package",
            default_value="xxx_resources_ft_sensor_gazebo_description",
            description="Package with the Gazebo world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file_path",
            default_value="",
            description="The path to the Gazebo world file inside the "
            "`gazebo_world_package`. If not specified, the default world file is used. "
            "If the force/torque sensing is enabled, the world with the force/torque "
            "sensor is used, otherwise the empty world is used.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
