r"""The full flight stack against a fake flight controller: no PX4, no Gazebo.

    ros2 launch autolivery_drone dry_run.launch.py
    ros2 action send_goal --feedback /autolivery/deliver \
        autolivery_interfaces/action/Deliver "{location: park}"

Runs mission_node, payload_node and planner_node exactly as on a vehicle, with fake_fcu
standing in for MAVROS + PX4 and the Gazebo gripper. Use it to check a new map, vehicle
profile or dropoff end to end in a couple of minutes, and in CI.
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
            'gripper': 'simulated',
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
            'battery_drain', default_value='0.0005',
            description='Battery fraction per second while armed (0.0005 = 33 min flight).'),
        DeclareLaunchArgument(
            'gripper_stuck', default_value='false',
            description='Make the fake gripper refuse to open, to exercise the abort path.'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_fake_fcu),
        stack,
    ])
