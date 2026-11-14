r"""The delivery drone in the Gazebo city, flown by PX4 SITL.

    export PX4_DIR=~/PX4-Autopilot        # built with: make px4_sitl
    ros2 launch autolivery_gazebo sim.launch.py vehicle:=quadcopter
    ros2 action send_goal --feedback /autolivery/deliver \
        autolivery_interfaces/action/Deliver "{location: rooftop_cafe}"

What starts, in order:
  gz sim            the urban_delivery world (generated from the city map)
  create            the vehicle (a PX4 model + DetachableJoint), then its parcel under it
  px4               SITL, attached to the spawned model (PX4_GZ_STANDALONE + MODEL_NAME)
  px4-param         sim-only parameter overrides once PX4 is up
  parameter_bridge  /clock and the gripper topics between Gazebo and ROS
  sim_traffic       other aircraft flying the map's traffic routes
  sim_weather       the wind in the world, and the weather report the flight stack gets
  autolivery        MAVROS + mission/payload/planner nodes, simulated gripper

Vehicles with a Gazebo model: quadcopter (x500), vtol (standard_vtol), fixed_wing
(rc_cessna, from the airstrip depot). The vehicle is spawned on its depot pad because the
flight stack takes PX4's local origin to be the depot.

The world is calm unless told otherwise:

    ros2 launch autolivery_gazebo sim.launch.py wind_speed:=6 wind_direction:=320 wind_gust:=9

blows 6 m/s from the north-west with gusts to 9 and reports it on /autolivery/weather, so a
wind over the vehicle's limit is refused; weather_report:=false blows without telling anyone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from autolivery_gazebo.city_map import position
from autolivery_gazebo.sim_models import (
    GRIPPER_TOPICS, parcel_sdf, parcel_spawn_position, vehicle_sdf, VEHICLES)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Sim-only PX4 overrides; real vehicles keep the airframe defaults.
#   NAV_DLL_ACT    every gz SITL airframe sets 2, which refuses to arm without a ground
#                  station (MAVROS identifies as an onboard controller, not a GCS), and
#                  there is none in the simulation.
#   FW_LND_USETER  fixed-wing landings use a rangefinder terrain estimate and abort (hold
#                  30 m above the runway) when there is none; the Gazebo rc_cessna has no
#                  rangefinder. A real plane should have one.
SIM_PX4_PARAMS = {
    'NAV_DLL_ACT': '0',
    'FW_LND_USETER': '0',
}


def _px4_param_script(bin_dir):
    """Shell loop that applies SIM_PX4_PARAMS once PX4 has finished booting.

    Setting them as soon as the PX4 socket answers is too early: the startup script imports
    saved parameters and runs the airframe afterwards. MAVLink is started at the very end of
    the startup script, so wait for it, then set and read each value back.
    """
    sets = ' && '.join(f'{bin_dir}/px4-param set {k} {v}' for k, v in SIM_PX4_PARAMS.items())
    checks = ' && '.join(
        f"{bin_dir}/px4-param show {k} | grep -q ': {v}$'" for k, v in SIM_PX4_PARAMS.items())
    return (
        'for i in $(seq 120); do sleep 1; '
        f'{bin_dir}/px4-mavlink status >/dev/null 2>&1 || continue; '
        f'({sets}) >/dev/null 2>&1; '
        f'if {checks}; then echo "sim PX4 parameters applied: {SIM_PX4_PARAMS}"; exit 0; fi; '
        'done; echo "could not apply sim PX4 parameters" >&2; exit 1')


def _load_vehicle(share_drone, vehicle):
    path = os.path.join(share_drone, 'config', 'vehicles', f'{vehicle}.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(f"unknown vehicle '{vehicle}' ({path} not found)")
    with open(path) as fh:
        params = yaml.safe_load(fh)['/**']['ros__parameters']
    sim_model = params['vehicle'].get('sim_model', '')
    if sim_model not in VEHICLES:
        simulated = []
        for f in sorted(os.listdir(os.path.dirname(path))):
            with open(os.path.join(os.path.dirname(path), f)) as fh:
                if yaml.safe_load(fh)['/**']['ros__parameters']['vehicle'].get(
                        'sim_model', '') in VEHICLES:
                    simulated.append(f[:-5])
        raise RuntimeError(
            f"vehicle '{vehicle}' has no Gazebo model; simulated vehicles: {', '.join(simulated)}")
    return params, VEHICLES[sim_model]


def _sim(context):
    share_drone = get_package_share_directory('autolivery_drone')
    share_gz = get_package_share_directory('autolivery_gazebo')
    vehicle = LaunchConfiguration('vehicle').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    px4_dir = os.path.expanduser(LaunchConfiguration('px4_dir').perform(context))
    map_arg = LaunchConfiguration('map').perform(context)
    map_file = map_arg if os.path.isabs(map_arg) else os.path.join(share_drone, map_arg)

    params, sim = _load_vehicle(share_drone, vehicle)
    with open(map_file) as fh:
        city = yaml.safe_load(fh)
    depot_name = LaunchConfiguration('depot').perform(context) or params.get('depot', 'main')
    if depot_name not in city['depots']:
        raise RuntimeError(f"depot '{depot_name}' is not in {map_file}")
    depot = position(city, city['depots'][depot_name])
    world_name = city['name']
    world_file = os.path.join(share_gz, 'worlds', f'{world_name}.sdf')

    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    for required in (px4_bin, px4_models, world_file):
        if not os.path.exists(required):
            raise RuntimeError(
                f'{required} not found. Set px4_dir (or PX4_DIR) to a PX4-Autopilot checkout '
                'built with `make px4_sitl`.')

    model_name = f'autolivery_{sim.base_model}'
    parcel_name = 'parcel'
    parcel_pos = parcel_spawn_position(sim, depot)

    gz_args = ['gz', 'sim', '-r', '-v', '2', world_file]
    if headless:
        gz_args.insert(3, '-s')
    gazebo = ExecuteProcess(cmd=gz_args, output='screen', name='gazebo')

    spawn_vehicle = Node(
        package='ros_gz_sim', executable='create', name='spawn_vehicle', output='screen',
        arguments=[
            '-world', world_name, '-name', model_name,
            '-string', vehicle_sdf(sim, model_name, parcel_name),
            '-x', str(depot[0]), '-y', str(depot[1]), '-z', str(depot[2])],
    )
    spawn_parcel = Node(
        package='ros_gz_sim', executable='create', name='spawn_parcel', output='screen',
        arguments=[
            '-world', world_name, '-name', parcel_name, '-string', parcel_sdf(sim, parcel_name),
            '-x', str(parcel_pos[0]), '-y', str(parcel_pos[1]), '-z', str(parcel_pos[2])],
    )

    px4_env = {
        'PX4_SYS_AUTOSTART': str(sim.airframe),
        'PX4_SIMULATOR': 'gz',
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODEL_NAME': model_name,
        'HEADLESS': '1',
    }
    px4 = ExecuteProcess(
        cmd=[px4_bin, '-d'], cwd=px4_dir, additional_env=px4_env,
        output=LaunchConfiguration('px4_output').perform(context), name='px4',
    )
    px4_params = ExecuteProcess(
        cmd=['bash', '-c', _px4_param_script(os.path.dirname(px4_bin))],
        output='screen', name='px4_params',
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f"{GRIPPER_TOPICS['detach']}@std_msgs/msg/Empty]gz.msgs.Empty",
            f"{GRIPPER_TOPICS['attach']}@std_msgs/msg/Empty]gz.msgs.Empty",
            f"{GRIPPER_TOPICS['state']}@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
    )
    traffic = Node(
        package='autolivery_gazebo', executable='sim_traffic_node', name='sim_traffic',
        output='screen',
        parameters=[{'use_sim_time': True, 'map_file': map_file, 'world': world_name}],
    )
    weather = Node(
        package='autolivery_gazebo', executable='sim_weather_node', name='sim_weather',
        output='screen',
        parameters=[{
            'use_sim_time': True, 'world': world_name,
            'wind_speed': float(LaunchConfiguration('wind_speed').perform(context)),
            'wind_direction': float(LaunchConfiguration('wind_direction').perform(context)),
            'wind_gust': float(LaunchConfiguration('wind_gust').perform(context)),
            'report': LaunchConfiguration('weather_report').perform(context) == 'true',
        }],
    )
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_drone, 'launch', 'autolivery.launch.py')),
        launch_arguments={
            'vehicle': vehicle,
            'gripper': 'simulated',
            'map': map_file,
            'depot': depot_name,
            'planner': LaunchConfiguration('planner').perform(context),
            'fcu_url': 'udp://:14540@127.0.0.1:14580',
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz').perform(context),
        }.items(),
    )

    resource_path = os.pathsep.join(
        p for p in (os.environ.get('GZ_SIM_RESOURCE_PATH', ''), px4_models) if p)
    actions = [
        # PX4's models (x500, standard_vtol, rc_cessna) are included by the vehicle wrappers.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        # The world must exist before anything is spawned into it; create retries its
        # service call, so starting it with Gazebo is enough.
        RegisterEventHandler(OnProcessStart(target_action=gazebo, on_start=[spawn_vehicle])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_vehicle, on_exit=[spawn_parcel])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_parcel, on_exit=[px4, px4_params])),
        # Without the autopilot or the world there is nothing to fly; stop everything rather
        # than leave the flight stack talking to a stale PX4 (e.g. one from an earlier run
        # still holding instance 0, which makes this one exit immediately).
        RegisterEventHandler(OnProcessExit(target_action=px4, on_exit=[
            LogInfo(msg='PX4 exited (is another SITL instance 0 still running?); shutting down'),
            EmitEvent(event=Shutdown(reason='PX4 exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        bridge,
        weather,
        stack,
    ]
    if LaunchConfiguration('traffic').perform(context) == 'true':
        actions.append(traffic)
    return actions


def generate_launch_description():
    px4_default = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle', default_value='quadcopter',
            description='Vehicle profile with a Gazebo model: quadcopter, vtol, fixed_wing.'),
        DeclareLaunchArgument(
            'px4_dir', default_value=px4_default,
            description='PX4-Autopilot checkout built with `make px4_sitl` (default $PX4_DIR).'),
        DeclareLaunchArgument(
            'map', default_value='maps/urban_delivery.yaml',
            description='City map; the world is worlds/<map name>.sdf in this package.'),
        DeclareLaunchArgument(
            'depot', default_value='', description="Depot to spawn at (default: the profile's)."),
        DeclareLaunchArgument('planner', default_value='astar', description='astar or rrt_star.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Gazebo without its GUI.'),
        DeclareLaunchArgument(
            'traffic', default_value='true', description='Fly the simulated traffic.'),
        DeclareLaunchArgument(
            'wind_speed', default_value='0.0', description='Mean wind in the world, m/s.'),
        DeclareLaunchArgument(
            'wind_direction', default_value='0.0',
            description='Direction the wind blows from, degrees clockwise from north.'),
        DeclareLaunchArgument(
            'wind_gust', default_value='0.0', description='Strongest gust, m/s.'),
        DeclareLaunchArgument(
            'weather_report', default_value='true',
            description='Report that wind on /autolivery/weather, as a depot station would.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        DeclareLaunchArgument(
            'px4_output', default_value='log',
            description='Where PX4 console output goes: log or screen.'),
        OpaqueFunction(function=_sim),
    ])
