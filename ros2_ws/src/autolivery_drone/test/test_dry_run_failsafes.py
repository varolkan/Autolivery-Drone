"""Refusals and operator aborts through the ROS graph, against the fake FCU."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from autolivery_interfaces.action import Deliver  # noqa: E402
from dry_run_support import ABORTED, CANCELED, DeliveryClient, dry_run_launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return dry_run_launch(vehicle='quadcopter')


class TestDryRunFailsafes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dry_run_failsafe_test')
        cls.client = DeliveryClient(cls.node)
        cls.client.wait_until_vehicle_ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_unknown_location_is_refused_with_the_known_ones(self):
        goal = Deliver.Goal()
        goal.location = 'the_moon'
        _, response = self.client.send(goal, timeout=10.0)
        self.assertEqual(response.status, ABORTED)
        self.assertIn('unknown location', response.result.message)
        self.assertIn('rooftop_cafe', response.result.message)
        self.assertFalse(response.result.delivered)

    def test_2_no_fly_zone_dropoff_is_refused(self):
        goal = Deliver.Goal()
        goal.dropoff.x, goal.dropoff.y = 170.0, -25.0  # hospital helipad zone
        goal.snap_to_surface = True
        _, response = self.client.send(goal, timeout=10.0)
        self.assertEqual(response.status, ABORTED)
        self.assertIn('blocked', response.result.message)

    def test_3_dropoff_inside_a_building_is_refused(self):
        goal = Deliver.Goal()
        goal.dropoff.x, goal.dropoff.y, goal.dropoff.z = 55.0, 50.0, 10.0  # in the office tower
        _, response = self.client.send(goal, timeout=10.0)
        self.assertEqual(response.status, ABORTED)
        self.assertIn('below the surface', response.result.message)

    def test_4_busy_then_cancel_returns_home_with_the_parcel(self):
        goal = Deliver.Goal()
        goal.location = 'park'
        handle = self.client.send_async(goal)
        self.assertTrue(self.client.spin_until(
            lambda: any(f.state == 'outbound' for f in self.client.feedback), 120.0))

        # A second delivery while the first is flying is refused, not queued or merged.
        second = Deliver.Goal()
        second.location = 'rooftop_cafe'
        _, busy = self.client.send(second, timeout=10.0)
        self.assertEqual(busy.status, ABORTED)
        self.assertIn('already in progress', busy.result.message)

        cancel = handle.cancel_goal_async()
        self.assertTrue(self.client.spin_until(cancel.done, 10.0))
        response = self.client.wait_result(handle, 300.0)
        self.assertEqual(response.status, CANCELED)
        self.assertFalse(response.result.delivered)
        self.assertIn('cancelled', response.result.message)

        self.assertTrue(self.client.spin_until(
            lambda: self.client.last_status().state == 'complete', 10.0))
        status = self.client.last_status()
        self.assertEqual(status.payload, 'attached')
        self.assertLess(abs(status.position.x), 2.0)
        self.assertLess(abs(status.position.y), 2.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
