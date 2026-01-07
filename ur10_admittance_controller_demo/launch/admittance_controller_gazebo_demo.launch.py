import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import (
    FindPackageShare,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

DEFAULT_FT_CONTROLLERS_FILE = "ur10_simulation_admittance_controller_wall.yaml"
DEFAULT_EFFORT_FT_CONTROLLERS_FILE = (
    "ur10_simulation_admittance_controller_wall_effort.yaml"
)


def launch_setup(context, *args, **kwargs):
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    initial_positions_package = LaunchConfiguration("initial_positions_package")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_effort_interfaces = LaunchConfiguration("enable_effort_interfaces")
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")
    plot_controller_state = LaunchConfiguration("plot_controller_state")
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")
    spawn_z = LaunchConfiguration("spawn_z")

    # If a controller file is not specified by the user, choose which one to use among default ones
    if controllers_file.perform(context) == DEFAULT_FT_CONTROLLERS_FILE:
        if enable_effort_interfaces.perform(context) == "true":
            controllers_file = DEFAULT_EFFORT_FT_CONTROLLERS_FILE
            initial_joint_controller = "pid_controller"

    gazebo_ros2_control_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xxx_resources_ur10_moveit_config"),
                    "launch",
                    "gazebo_ros2_control_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "safety_limits": "true",
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "initial_joint_controller": initial_joint_controller,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": tf_prefix,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "launch_rviz": launch_rviz,
            "enable_ft_sensing": "true",
            "enable_effort_interfaces": enable_effort_interfaces,
            "ft_plot": "false",
            "ee_config_package": ee_config_package,
            "ee_config_file": ee_config_file,
            "initial_positions_package": initial_positions_package,
            "initial_positions_file": initial_positions_file,
            "gazebo_world_package": gazebo_world_package,
            "gazebo_world_file_path": gazebo_world_file,
            "spawn_z": spawn_z,
        }.items(),
    )

    cartesian_pose_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_pose_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    reference_generator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "task_space_reference_generator",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_reference_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[reference_generator_spawner],
        )
    )

    layout_path_plotjuggler = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "admittance_controller_state_plots.xml",
        ]
    )

    plotjuggler_starter = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path_plotjuggler],
        condition=IfCondition(plot_controller_state),
    )

    if enable_effort_interfaces.perform(context) == "true":
        initial_positions_path = PathJoinSubstitution(
            [FindPackageShare(runtime_config_package), "config", initial_positions_file]
        )

        with open(initial_positions_path.perform(context), "r") as f:
            initial_positions_params = yaml.safe_load(f)

        initial_positions = []
        for joint_name, joint_position in initial_positions_params.items():
            initial_positions.append(joint_position)

        publish_initial_positions = ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "pub",
                "/pid_controller/reference",
                "control_msgs/msg/MultiDOFCommand",
                "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values:"
                + str(initial_positions)
                + "}",
                "-t",
                "2",  # Duration of the publication
            ],
            output="screen",
        )

        delay_cartesian_pose_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=publish_initial_positions,
                on_exit=[
                    TimerAction(period=2.0, actions=[cartesian_pose_controller_spawner])
                ],
            )
        )

        delay_admittance_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=cartesian_pose_controller_spawner,
                on_exit=[admittance_controller_spawner],
            )
        )

        nodes_to_start = [
            gazebo_ros2_control_demo_launch,
            publish_initial_positions,
            delay_cartesian_pose_controller_spawner,
            delay_admittance_controller_spawner,
            delay_reference_after_controller_spawner,
            plotjuggler_starter,
        ]
    else:
        delay_admittance_controller_spawner = TimerAction(
            period=5.0,
            actions=[admittance_controller_spawner],
        )

        nodes_to_start = [
            gazebo_ros2_control_demo_launch,
            delay_admittance_controller_spawner,
            delay_reference_after_controller_spawner,
            plotjuggler_starter,
        ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
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
            default_value="ur10_admittance_controller_demo",
            description="Package with the controller's configuration in `config` folder. \
            Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=DEFAULT_FT_CONTROLLERS_FILE,
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="xxx_resources_ur10_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
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
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="ur10_admittance_controller_demo",
            description="Package with the RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="config/ur10_moveit.rviz",
            description="RViz configuration file.",
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
            "initial_positions_package",
            default_value="ur10_admittance_controller_demo",
            description="Package with the initial joint positions file under the `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions_wall.yaml",
            description="Initial joint positions of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_package",
            default_value="xxx_resources_ft_sensor_gazebo_description",
            description="Package with the world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="world/world_with_ft_sensor_and_wall.sdf",
            description="World file related to the `gazebo_world_package`.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "number_of_samples_for_computing_bias",
            default_value="0",
            description="This parameter has a dual purpose: \
                            - If set to a value greater than 0, enable the bias calculation for the Gazebo F/T sensor plugin. \
                            - Its value specifies the number of iterations to consider when estimating the bias along the axes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "plot_controller_state",
            default_value="true",
            description="Enables the launch of the PlotJuggler tool for real-time visualization of F/T plots.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_package",
            default_value="ur10_admittance_controller_demo",
            description="Package with the end-effector configuration file under the `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_file",
            default_value="config/ft_sensor_and_short_handle_ee_config.yaml",
            description="End-effector configuration file. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z coordinate of the robot spawn position.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
