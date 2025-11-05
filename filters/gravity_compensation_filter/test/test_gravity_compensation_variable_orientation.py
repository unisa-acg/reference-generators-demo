from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import pytest
from unittest import TestCase


@pytest.mark.launch_test
def generate_test_description():
    gravity_compensation_configuration = PathJoinSubstitution(
        [
            FindPackageShare("gravity_compensation_filter"),
            "test",
            "config",
            "gravity_compensation_variable_orientation_config.yaml",
        ]
    )

    # The node to test
    test_gravity_compensation_node = Node(
        package="gravity_compensation_filter",
        executable="gravity_compensation_variable_orientation_test",
        name="test_gravity_compensation_variable_orientation",
        output="screen",
        parameters=[gravity_compensation_configuration],
    )

    # Launch the processes and execute tests
    return (
        LaunchDescription(
            [
                test_gravity_compensation_node,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {
            "test_gravity_compensation_node": test_gravity_compensation_node,
        },
    )


class TestTerminatingProcessStops(TestCase):
    def test_gtest_run_complete(self, proc_info, test_gravity_compensation_node):
        proc_info.assertWaitForShutdown(
            process=test_gravity_compensation_node, timeout=4000.0
        )


@post_shutdown_test()
class TestOutcome(TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
