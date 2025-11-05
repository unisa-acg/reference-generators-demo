from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc


import pytest

from unittest import TestCase


@pytest.mark.launch_test
def generate_test_description():
    # Input arguments
    parameters = {
        "number_of_stable_samples_for_bias": 10,
        "tolerance": 0.0000001,
    }

    # The node to test
    test_gravity_compensation_node = Node(
        package="unbiasing_filter",
        executable="gravity_compensation_fixed_orientation_test",
        name="test_gravity_compensation_fixed_orientation",
        parameters=[parameters],
        output="screen",
    )

    # Launch the processes and execute tests
    return (
        LaunchDescription(
            [test_gravity_compensation_node, KeepAliveProc(), ReadyToTest()]
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
