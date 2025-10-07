from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc
from launch.substitutions import Command, PathJoinSubstitution

import pytest
from unittest import TestCase


@pytest.mark.launch_test
def generate_test_description():

    # URDF file
    description_package = FindPackageShare("ros2_control_test_assets")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "test_hardware_components.urdf"]
    )

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro ", description_file]), value_type=str
    )

    # Input arguments
    parameters = {
        "plugin_package": "kinematics_interface",
        "kinematics_interface_plugin_name": "kinematics_interface_kdl/KinematicsInterfaceKDL",
        "tip_link": "tool_link",
        "robot_description": robot_description,
    }

    # The node to test
    test_kinematics_node = Node(
        package="acg_common_libraries",
        executable="kinematics_test",
        name="test_kinematics_node",
        parameters=[parameters],
        output="screen",
    )

    # Launch the processes and execute tests
    return (
        LaunchDescription(
            [
                test_kinematics_node,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {"test_kinematics_node": test_kinematics_node},
    )


class TestTerminatingProcessStops(TestCase):
    def test_gtest_run_complete(self, proc_info, test_kinematics_node):
        proc_info.assertWaitForShutdown(process=test_kinematics_node, timeout=4000.0)


@post_shutdown_test()
class TestOutcome(TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
