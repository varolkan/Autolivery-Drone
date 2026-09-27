"""A whole delivery through the ROS graph: Deliver goal -> fake FCU flight -> parcel dropped.

Exercises everything the unit tests cannot: parameter loading from the shipped YAML, the
MAVROS topic and service contract, the OFFBOARD/arming handshake over real services, the
gripper round trip through payload_node, and the action lifecycle.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from autolivery_interfaces.action import Deliver  # noqa: E402
from dry_run_support import DeliveryClient, dry_run_launch, SUCCEEDED  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return dry_run_launch(vehicle='quadcopter')


class TestDryRunDelivery(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dry_run_delivery_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_rooftop_delivery_and_return(self):
        client = DeliveryClient(self.node)
        client.wait_until_vehicle_ready()

        goal = Deliver.Goal()
        goal.location = 'rooftop_cafe'
        _, response = client.send(goal, timeout=480.0)

        self.assertEqual(response.status, SUCCEEDED, response.result.message)
        self.assertTrue(response.result.success)
        self.assertTrue(response.result.delivered)
        self.assertGreater(response.result.distance_flown, 250.0)
        self.assertGreater(response.result.flight_time, 30.0)

        states = [f.state for f in client.feedback]
        for expected in ('takeoff', 'outbound', 'descend', 'release', 'return', 'land'):
            self.assertIn(expected, states)
        self.assertLess(states.index('outbound'), states.index('return'))

        # The parcel came off, and the vehicle is back down with its motors off.
        self.assertTrue(client.spin_until(
            lambda: client.last_status() is not None and client.last_status().state == 'complete'
            and not client.last_status().armed, 10.0))
        status = client.last_status()
        self.assertEqual(status.payload, 'released')
        self.assertLess(abs(status.position.x), 2.0)
        self.assertLess(abs(status.position.y), 2.0)
        self.assertLess(status.position.z, 0.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
