# Based on Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/launch/ur_sim_control.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Set the main controller to activate
    CONTROLLER_NAME = "gravity_compensation_pd_controller"

    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_config_file = LaunchConfiguration("controllers_config_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file_path = LaunchConfiguration("gazebo_world_file_path")

    gazebo_ros2_control_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_fer_moveit_config"),
                    "launch",
                    "gazebo_ros2_control_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_config_file,
            "controller": CONTROLLER_NAME,
            "launch_rviz": launch_rviz,
            "tf_prefix": tf_prefix,
            "enable_effort_interfaces": "true",
            "hand": "false",
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "gazebo_world_package": gazebo_world_package,
            "gazebo_world_file_path": gazebo_world_file_path,
        }.items(),
    )

    delay_joint_space_reference_generator_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_space_reference_generator",
                    "--controller-manager",
                    "/controller_manager",
                ],
                name="joint_space_reference_generator_spawner",
                output="screen",
            )
        ],
    )

    # This file contains the parameters for the bridge between ROS2 and
    # Ignition Gazebo without putting a condition as done for the torque
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

    nodes_to_start = [
        gazebo_ros2_control_demo_launch,
        delay_joint_space_reference_generator_spawner,
        gazebo_ros_bridge_cmd,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="fer_gravity_compensation_pd_controller_demo",
            description="Package with the controller's configuration. "
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_file",
            default_value="fer_gc_pd_simulation.yaml",
            description="YAML file with the controllers configuration. "
            "The file is searched under the `config` folder of the specified "
            "`runtime_config_package` or in the provided relative path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup. "
            "If changed than also joint names in the controllers' configuration have "
            "to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="fer_gravity_compensation_pd_controller_demo",
            description="Package with the RViz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="gravity_compensation_pd_controller.rviz",
            description="RViz config file to use.",
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
