r"""The full flight stack against a fake flight controller: no PX4, no Gazebo.

    ros2 launch autolivery_drone dry_run.launch.py
    ros2 action send_goal --feedback /autolivery/deliver \
        autolivery_interfaces/action/Deliver "{location: park}"

Runs mission_node, payload_node and planner_node exactly as on a vehicle, with fake_fcu
standing in for MAVROS + PX4 and the Gazebo gripper. Use it to check a new map, vehicle
profile or dropoff end to end in a couple of minutes, and in CI.

`gripper:=mavlink_gripper` (or servo, electromagnet, winch) runs the hardware gripper path
instead: payload_node sends the MAVLink commands and fake_fcu answers them like an
autopilot. `gripper_rejections:=N` makes it refuse the first N, to watch the retry.

`wind_speed:=6 wind_direction:=320 wind_gust:=9` blows on the fake vehicle (m/s, the
direction it comes from in degrees) and is reported on /autolivery/weather, so the
vehicle's weather limits can be seen at work; `weather_report:=false` keeps the wind but
not the report.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fake_fcu(context):
    share = get_package_share_directory('autolivery_drone')
    vehicle = LaunchConfiguration('vehicle').perform(context)
    params = [
        os.path.join(share, 'config', 'mission.yaml'),
        os.path.join(share, 'config', 'vehicles', f'{vehicle}.yaml'),
        {
            'map_file': LaunchConfiguration('map').perform(context),
            'battery_drain_per_s': float(LaunchConfiguration('battery_drain').perform(context)),
            'gripper_stuck': LaunchConfiguration('gripper_stuck').perform(context) == 'true',
            'gripper_rejections': int(LaunchConfiguration('gripper_rejections').perform(context)),
            'wind_speed': float(LaunchConfiguration('wind_speed').perform(context)),
            'wind_direction': float(LaunchConfiguration('wind_direction').perform(context)),
            'wind_gust': float(LaunchConfiguration('wind_gust').perform(context)),
            'report_weather': LaunchConfiguration('weather_report').perform(context) == 'true',
        },
    ]
    depot = LaunchConfiguration('depot').perform(context)
    if depot:
        params.append({'depot': depot})
    return [Node(
        package='autolivery_drone', executable='fake_fcu_node', name='fake_fcu',
        output='screen', parameters=params, emulate_tty=True,
    )]


def generate_launch_description():
    share = get_package_share_directory('autolivery_drone')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'autolivery.launch.py')),
        launch_arguments={
            'vehicle': LaunchConfiguration('vehicle'),
            'gripper': LaunchConfiguration('gripper'),
            'map': LaunchConfiguration('map'),
            'depot': LaunchConfiguration('depot'),
            'planner': LaunchConfiguration('planner'),
            'use_mavros': 'false',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='quadcopter'),
        DeclareLaunchArgument('map', default_value='maps/urban_delivery.yaml'),
        DeclareLaunchArgument('depot', default_value=''),
        DeclareLaunchArgument('planner', default_value='astar'),
        DeclareLaunchArgument(
            'gripper', default_value='simulated',
            description='Gripper config; anything but simulated is driven over MAVLink.'),
        DeclareLaunchArgument(
            'gripper_rejections', default_value='0',
            description='Gripper commands the fake autopilot refuses before accepting one.'),
        DeclareLaunchArgument(
            'battery_drain', default_value='0.0005',
            description='Battery fraction per second while armed (0.0005 = 33 min flight).'),
        DeclareLaunchArgument(
            'gripper_stuck', default_value='false',
            description='Make the fake gripper refuse to open, to exercise the abort path.'),
        DeclareLaunchArgument(
            'wind_speed', default_value='0.0', description='Mean wind on the fake vehicle, m/s.'),
        DeclareLaunchArgument(
            'wind_direction', default_value='0.0',
            description='Direction the wind blows from, degrees clockwise from north.'),
        DeclareLaunchArgument(
            'wind_gust', default_value='0.0', description='Strongest gust, m/s.'),
        DeclareLaunchArgument(
            'weather_report', default_value='true',
            description='Report that wind on /autolivery/weather, as a depot station would.'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_fake_fcu),
        stack,
    ])
