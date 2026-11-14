"""The weather limits through the ROS graph.

The test is the depot's weather station: it reports on /autolivery/weather and the fake
vehicle flies in still air, so what is checked is what the mission does with a report. A
gale and rain refuse a delivery before anything spins up, and wind that picks up with the
vehicle on its way out brings it home with the parcel still on board.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from autolivery_interfaces.action import Deliver  # noqa: E402
from autolivery_interfaces.msg import Weather  # noqa: E402
from dry_run_support import ABORTED, DeliveryClient, dry_run_launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return dry_run_launch(vehicle='quadcopter', weather_report='false')


class TestDryRunWeather(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dry_run_weather_test')
        cls.client = DeliveryClient(cls.node)
        cls.report = Weather()
        cls.station = cls.node.create_publisher(Weather, '/autolivery/weather', 10)
        cls.node.create_timer(0.2, cls.publish_report)
        cls.set_weather(wind=3.0, gust=5.0)
        cls.client.wait_until_vehicle_ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_report(cls):
        cls.report.header.stamp = cls.node.get_clock().now().to_msg()
        cls.station.publish(cls.report)

    @classmethod
    def set_weather(cls, wind, gust, precipitation=0.0, temperature=12.0):
        cls.report.wind_speed = wind
        cls.report.wind_direction = 320.0
        cls.report.wind_gust = gust
        cls.report.precipitation = precipitation
        cls.report.temperature = temperature

    def wait_for_wind(self, speed):
        """Wait until the mission is working with the wind the station now reports."""
        self.assertTrue(self.client.spin_until(
            lambda: self.client.statuses
            and math.isclose(self.client.last_status().wind, speed, abs_tol=1e-3), 20.0),
            f'the mission never took in a wind of {speed} m/s: {self.client.last_status()}')

    def test_1_gale_is_refused_on_the_ground(self):
        self.set_weather(wind=15.0, gust=19.0)
        self.wait_for_wind(15.0)
        goal = Deliver.Goal()
        goal.location = 'park'
        _, response = self.client.send(goal, timeout=10.0)
        self.assertEqual(response.status, ABORTED)
        self.assertEqual(
            response.result.message, 'wind of 15.0 m/s is over the quadcopter limit of 8.0 m/s')
        self.assertFalse(response.result.delivered)
        self.assertFalse(self.client.last_status().armed)

    def test_2_rain_is_refused_on_the_ground(self):
        self.set_weather(wind=3.0, gust=5.0, precipitation=3.0)
        self.wait_for_wind(3.0)
        goal = Deliver.Goal()
        goal.location = 'park'
        _, response = self.client.send(goal, timeout=10.0)
        self.assertEqual(response.status, ABORTED)
        self.assertIn('precipitation of 3.0 mm/h', response.result.message)
        self.assertFalse(self.client.last_status().armed)

    def test_3_wind_picking_up_in_flight_brings_the_parcel_home(self):
        self.set_weather(wind=4.0, gust=6.0)
        self.wait_for_wind(4.0)
        goal = Deliver.Goal()
        goal.location = 'park'
        handle = self.client.send_async(goal)
        self.assertTrue(self.client.spin_until(
            lambda: any(f.state == 'outbound' for f in self.client.feedback), 120.0))

        self.set_weather(wind=12.0, gust=16.0)
        response = self.client.wait_result(handle, 300.0)
        self.assertEqual(response.status, ABORTED)
        self.assertFalse(response.result.delivered)
        self.assertIn('wind of 12.0 m/s is over the quadcopter limit', response.result.message)
        self.assertIn('abandoning delivery', response.result.message)

        states = [f.state for f in self.client.feedback]
        self.assertNotIn('release', states)
        self.assertTrue(self.client.spin_until(
            lambda: self.client.last_status().state == 'complete'
            and not self.client.last_status().armed, 10.0))
        status = self.client.last_status()
        self.assertEqual(status.payload, 'attached')
        self.assertLess(abs(status.position.x), 2.0)
        self.assertLess(abs(status.position.y), 2.0)
        self.assertLess(status.position.z, 0.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
