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
    controllers_config_file = LaunchConfiguration("controllers_config_file")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    robot_ip = LaunchConfiguration("robot_ip")

    franka_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": "fer",
            "load_gripper": "false",
            "use_rviz": "false",
            "controller_file_package": "fer_gravity_compensation_pd_controller_demo",
            "controller_file_path": controllers_config_file,
            "initial_joint_controller": CONTROLLER_NAME,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(rviz_config_package),
                    "config",
                    rviz_config_file,
                ]
            ),
        ],
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
            FindPackageShare("xxx_resources_fer_moveit_config"),
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
        franka_bringup_launch,
        rviz_node,
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
            default_value="config/real_fer_gc_pd_simulation.yaml",
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
            "robot_ip",
            description="IP address by which the robot can be reached.",
            default_value="192.168.1.1",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
