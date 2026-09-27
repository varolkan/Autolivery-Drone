"""A fixed-wing air drop through the ROS graph, against the fake FCU.

Covers what only the plane does: the runway landing pattern is uploaded over the MAVROS
mission API before any delivery is accepted, takeoff is AUTO.TAKEOFF from the airstrip, the
parcel is released in forward flight, and the flight ends with RTL flying that pattern.
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
    return dry_run_launch(vehicle='fixed_wing')


class TestDryRunFixedWing(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dry_run_fixed_wing_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_air_drop_at_the_farm_and_runway_landing(self):
        client = DeliveryClient(self.node)
        client.wait_until_vehicle_ready()

        goal = Deliver.Goal()
        goal.location = 'farm'
        # The landing pattern upload happens once the FCU is connected; a goal sent before it
        # is accepted is refused with a reason rather than flown without a way to land.
        for _ in range(10):
            _, response = client.send(goal, timeout=600.0)
            if 'landing pattern' not in response.result.message:
                break
            client.spin_until(lambda: False, 2.0)

        self.assertEqual(response.status, SUCCEEDED, response.result.message)
        self.assertTrue(response.result.delivered)
        states = [f.state for f in client.feedback]
        self.assertIn('outbound', states)
        self.assertNotIn('descend', states)  # air drop: no hover over the target
        self.assertIn('land', states)
        self.assertTrue(client.spin_until(
            lambda: client.last_status().state == 'complete'
            and not client.last_status().armed, 10.0))
        self.assertEqual(client.last_status().payload, 'released')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
