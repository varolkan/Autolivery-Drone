"""A delivery with a hardware gripper (PX4 payload deliverer over MAVLink), fake autopilot.

No payload sensor here, so the only evidence that the parcel left is the autopilot's answer
to the gripper command. The fake autopilot refuses the first one: the mission must notice,
try again, and only report a delivery for the command that was accepted.
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
from std_msgs.msg import String  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return dry_run_launch(vehicle='quadcopter', gripper='mavlink_gripper', gripper_rejections=1)


class TestDryRunMavlinkGripper(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dry_run_mavlink_gripper_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_refused_release_is_retried_before_the_delivery_counts(self):
        client = DeliveryClient(self.node)
        # What the fake vehicle's gripper is actually holding, as Gazebo would report it.
        parcel = []
        self.node.create_subscription(
            String, '/autolivery/gripper/state', lambda m: parcel.append(m.data), 10)
        client.wait_until_vehicle_ready(payload='unknown')

        goal = Deliver.Goal()
        goal.location = 'park'
        _, response = client.send(goal, timeout=480.0)

        self.assertEqual(response.status, SUCCEEDED, response.result.message)
        self.assertTrue(response.result.delivered)

        details = [f.detail for f in client.feedback] + [s.detail for s in client.statuses]
        self.assertTrue(
            any('release failed' in d or 'release attempt failed' in d for d in details),
            'the refused gripper command went unnoticed')
        # While the first command was being refused the parcel was still on board...
        release = [s for s in client.statuses if s.state == 'release']
        self.assertGreater(len(release), 20, 'the retry period was not waited out')
        # ...and it did come off in the end.
        self.assertTrue(client.spin_until(lambda: parcel and parcel[-1] == 'detached', 5.0))
        self.assertIn('attached', parcel)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
