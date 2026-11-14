r"""Bring up the delivery flight stack for one vehicle.

    ros2 launch autolivery_drone autolivery.launch.py vehicle:=quadcopter gripper:=servo \
        fcu_url:=/dev/ttyACM0:921600 accept_risk:=true

Every launch, real or simulated, first prints the license terms (LICENSE.md). A real vehicle
is only flown with accept_risk:=true: the operator confirms having read the safety notice and
the license, accepts them, and takes sole responsibility for the flight. Without it nothing is
started. The dry-run and Gazebo launches include this one with simulation:=true and need no
such step.

Nodes:
  mavros             -- MAVLink link to PX4 (skipped with use_mavros:=false)
  mission_node       -- runs Deliver goals: plans, flies offboard, releases, returns
  payload_node       -- drives the gripper and reports whether the parcel is still on
  planner_node       -- plan_path service and the city map for RViz
  adsb_traffic_node  -- ADS-B traffic as avoidance obstacles (adsb:=true)

Then, from anywhere on the ROS graph:

    ros2 action send_goal --feedback /autolivery/deliver \
        autolivery_interfaces/action/Deliver "{location: rooftop_cafe}"

Configuration is layered: config/mission.yaml, then config/vehicles/<vehicle>.yaml, then
config/grippers/<gripper>.yaml, then the individual overrides below. The vehicle must be
powered up on its depot pad: the autopilot's local origin is taken to be the depot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SAFETY_NOTICE = (
    'Autolivery-Drone is experimental software, tested in simulation only and not certified for '
    'flight. It can fail in ways that crash the vehicle, drop the parcel in the wrong place, '
    'damage property or hurt people. Whoever flies it does so at their own risk and is solely '
    'responsible for the flight, for operating safely and lawfully, and for any authorisation it '
    'needs (see the safety notice in README.md and LICENSE.md).')

# The same terms as the Required Notice at the top of LICENSE.md.
LICENSE_NOTICE = (
    'Autolivery-Drone: This software is licensed under the PolyForm Noncommercial License 1.0.0 '
    'with Additional Conditions. Commercial use, military use and use in weapons are all '
    'forbidden, and anyone who shares or publishes the software, or anything made with it, must '
    'credit varolkan. Using it means accepting those terms: read LICENSE.md.')


def _operator_check(context):
    """Refuse to fly a real vehicle until the operator has accepted the license and the risk."""
    if LaunchConfiguration('simulation').perform(context) == 'true':
        return []
    if LaunchConfiguration('accept_risk').perform(context) != 'true':
        raise RuntimeError(
            SAFETY_NOTICE + ' To fly a real vehicle, confirm that you have read this and '
            'LICENSE.md and accept them, and that responsibility, by adding accept_risk:=true to '
            'the launch command.')
    accepted = ('operator accepted the license (LICENSE.md) and responsibility for this flight '
                '(accept_risk:=true). ')
    return [LogInfo(msg=accepted + SAFETY_NOTICE)]


def _stack(context):
    share = get_package_share_directory('autolivery_drone')
    vehicle = LaunchConfiguration('vehicle').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)
    vehicle_file = os.path.join(share, 'config', 'vehicles', f'{vehicle}.yaml')
    gripper_file = os.path.join(share, 'config', 'grippers', f'{gripper}.yaml')
    for kind, path in (('vehicle', vehicle_file), ('gripper', gripper_file)):
        if not os.path.isfile(path):
            available = sorted(
                f[:-5] for f in os.listdir(os.path.dirname(path)) if f.endswith('.yaml'))
            raise RuntimeError(
                f"unknown {kind} '{os.path.basename(path)[:-5]}'; "
                f"available: {', '.join(available)}")

    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        'map_file': LaunchConfiguration('map').perform(context),
        'planner.algorithm': LaunchConfiguration('planner').perform(context),
    }
    depot = LaunchConfiguration('depot').perform(context)
    if depot:
        overrides['depot'] = depot
    params = [os.path.join(share, 'config', 'mission.yaml'), vehicle_file, gripper_file, overrides]

    actions = [
        Node(
            package='autolivery_drone', executable='mission_node', name='mission_node',
            output='screen', parameters=params, emulate_tty=True,
        ),
        Node(
            package='autolivery_drone', executable='payload_node', name='payload_node',
            output='screen', parameters=params, emulate_tty=True,
        ),
        Node(
            package='autolivery_drone', executable='planner_node', name='planner_node',
            output='screen', parameters=params, emulate_tty=True,
        ),
        Node(
            package='autolivery_drone', executable='adsb_traffic_node', name='adsb_traffic_node',
            output='screen', parameters=params, emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('adsb')),
        ),
    ]
    return actions


def generate_launch_description():
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch')),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            # The MAVROS node is itself called "mavros"; pushing it into a "mavros"
            # namespace as well (px4.launch's default) doubles every topic to
            # /mavros/mavros/... Root namespace gives the documented /mavros/state layout.
            'namespace': '/',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mavros')),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(
            get_package_share_directory('autolivery_drone'), 'rviz', 'autolivery.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle', default_value='quadcopter',
            description='Vehicle profile in config/vehicles/ (quadcopter, hexacopter, '
                        'octocopter, coaxial_octocopter, helicopter, vtol, fixed_wing).'),
        DeclareLaunchArgument(
            'gripper', default_value='mavlink_gripper',
            description='Gripper in config/grippers/ (simulated, mavlink_gripper, servo, '
                        'electromagnet, winch).'),
        DeclareLaunchArgument(
            'map', default_value='maps/urban_delivery.yaml',
            description='City map, relative to this package or absolute.'),
        DeclareLaunchArgument(
            'depot', default_value='',
            description='Depot in the map the vehicle starts from. Empty: the vehicle '
                        "profile's choice, or 'main'."),
        DeclareLaunchArgument(
            'planner', default_value='astar', description='Global planner: astar or rrt_star.'),
        DeclareLaunchArgument(
            'fcu_url', default_value='udp://:14540@127.0.0.1:14580',
            description='MAVLink connection to PX4. The default is PX4 SITL instance 0; on a '
                        'companion computer use the serial port, e.g. /dev/ttyACM0:921600.'),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='Optional MAVLink forward to a ground station, e.g. udp://@10.0.0.2.'),
        DeclareLaunchArgument(
            'use_mavros', default_value='true',
            description='Start MAVROS. Off when something else provides the mavros API.'),
        DeclareLaunchArgument(
            'adsb', default_value='false',
            description='Avoid ADS-B traffic reported through MAVROS (needs mavros_extras).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use /clock (Gazebo).'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        DeclareLaunchArgument(
            'accept_risk', default_value='false',
            description='Must be true to fly a real vehicle: you confirm that you have read the '
                        'safety notice and LICENSE.md, accept them, and accept sole '
                        'responsibility for the flight.'),
        DeclareLaunchArgument(
            'simulation', default_value='false',
            description='Set by the dry-run and Gazebo launches; no vehicle is flown for real.'),
        # Every launch, real or simulated, shows the license terms first.
        LogInfo(msg=LICENSE_NOTICE),
        # Before anything is started, MAVROS included.
        OpaqueFunction(function=_operator_check),
        mavros,
        OpaqueFunction(function=_stack),
        rviz,
    ])
