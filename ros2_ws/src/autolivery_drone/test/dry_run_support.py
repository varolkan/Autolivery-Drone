"""Shared pieces of the dry-run launch tests: the launch description and an action client."""

import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from autolivery_interfaces.action import Deliver
from autolivery_interfaces.msg import MissionStatus
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient


def dry_run_launch(**arguments):
    share = get_package_share_directory('autolivery_drone')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'dry_run.launch.py')),
            launch_arguments={k: str(v) for k, v in arguments.items()}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class DeliveryClient:
    """Deliver action client that records feedback and mission status while it waits."""

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, Deliver, '/autolivery/deliver')
        self.feedback = []
        self.statuses = []
        node.create_subscription(
            MissionStatus, '/autolivery/mission/status', self.statuses.append, 10)

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return predicate()

    def wait_until_vehicle_ready(self, timeout=90.0, payload='attached'):
        """Wait for what an operator checks before dispatching: link up, parcel loaded.

        Grippers without a payload sensor report 'unknown'; pass that for them.
        """
        assert self.client.wait_for_server(timeout_sec=timeout), 'Deliver action server missing'
        assert self.spin_until(
            lambda: self.statuses and self.statuses[-1].connected
            and self.statuses[-1].payload == payload, timeout), \
            f'vehicle never became ready: {self.last_status()}'

    def send(self, goal, timeout):
        """Send a goal and wait for its result. Returns (goal_handle, result response)."""
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback))
        assert self.spin_until(future.done, 30.0), 'goal was never answered'
        handle = future.result()
        assert handle.accepted, 'goals are always accepted; refusals come back as results'
        result = handle.get_result_async()
        assert self.spin_until(result.done, timeout), f'no result within {timeout} s'
        return handle, result.result()

    def send_async(self, goal):
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback))
        assert self.spin_until(future.done, 30.0), 'goal was never answered'
        return future.result()

    def wait_result(self, handle, timeout):
        result = handle.get_result_async()
        assert self.spin_until(result.done, timeout), f'no result within {timeout} s'
        return result.result()

    def last_status(self):
        return self.statuses[-1] if self.statuses else None


SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
ABORTED = GoalStatus.STATUS_ABORTED
CANCELED = GoalStatus.STATUS_CANCELED
