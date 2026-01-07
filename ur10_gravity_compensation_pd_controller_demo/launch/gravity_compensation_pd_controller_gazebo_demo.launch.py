# Based on Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/launch/ur_sim_control.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
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

    controllers_config_file = PathJoinSubstitution(["config", controllers_config_file])

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
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_config_file,
            "initial_joint_controller": CONTROLLER_NAME,
            "launch_rviz": launch_rviz,
            "tf_prefix": tf_prefix,
            "enable_effort_interfaces": "true",
        }.items(),
    )

    delay_jtc_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                name="joint_trajectory_controller_spawner",
                output="screen",
            )
        ],
    )

    nodes_to_start = [
        gazebo_ros2_control_demo_launch,
        delay_jtc_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur10_gravity_compensation_pd_controller_demo",
            description="Package with the controller's configuration. "
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_file",
            default_value="ur10_gc_pd_simulation.yaml",
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
            "initial_joint_controller",
            default_value="gravity_compensation_pd_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
