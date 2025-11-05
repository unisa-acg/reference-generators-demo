from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_ur10_moveit_config"),
                    "launch",
                    "ur10_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": "192.168.1.6",
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "initial_joint_controller": initial_joint_controller,
            "launch_rviz": launch_rviz,
            "ee_config_package": ee_config_package,
            "ee_config_file": ee_config_file,
            "enable_ft_sensing": "true",
        }.items(),
    )

    return [
        ur_control_launch,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur10_admittance_controller_demo",
            description="Package with the controllers configuration in `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur10_admittance_controller_phri.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="acg_resources_ur10_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
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
            "initial_joint_controller",
            default_value="cartesian_pose_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
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
            default_value="config/robotiq_fts150_and_chalk_holder_ee_config.yaml",
            description="End-effector configuration file. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
