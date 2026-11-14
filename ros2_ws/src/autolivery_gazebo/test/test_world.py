"""The Gazebo world and simulated vehicles agree with the city map and with PX4's models."""

import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from autolivery_gazebo.city_map import position, surface_height
from autolivery_gazebo.sim_models import parcel_sdf, parcel_spawn_position, vehicle_sdf, VEHICLES
import pytest
import yaml

SOURCE = os.environ.get(
    'AUTOLIVERY_GAZEBO_SOURCE', os.path.join(os.path.dirname(__file__), '..'))

MAP = os.path.join(get_package_share_directory('autolivery_drone'), 'maps', 'urban_delivery.yaml')
WORLD = os.path.join(SOURCE, 'worlds', 'urban_delivery.sdf')
GENERATOR = os.path.join(SOURCE, 'scripts', 'generate_world.py')


@pytest.fixture(scope='module')
def city():
    with open(MAP) as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope='module')
def world():
    return ET.parse(WORLD).getroot().find('world')


def models(world):
    return {m.get('name'): m for m in world.findall('model')}


def floats(text):
    return [float(v) for v in text.split()]


def test_committed_world_matches_the_map():
    result = subprocess.run(
        [sys.executable, GENERATOR, '--check', MAP, WORLD], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_world_name_and_geodetic_origin_match_the_map(city, world):
    assert world.get('name') == city['name']
    sc = world.find('spherical_coordinates')
    assert float(sc.find('latitude_deg').text) == pytest.approx(city['origin']['latitude'])
    assert float(sc.find('longitude_deg').text) == pytest.approx(city['origin']['longitude'])
    assert sc.find('world_frame_orientation').text == 'ENU'


def test_px4_sensor_systems_are_loaded(world):
    loaded = {p.get('filename') for p in world.findall('plugin')}
    for required in ('gz-sim-physics-system', 'gz-sim-imu-system', 'gz-sim-air-pressure-system',
                     'gz-sim-magnetometer-system', 'gz-sim-navsat-system',
                     'gz-sim-air-speed-system', 'gz-sim-user-commands-system'):
        assert required in loaded


def test_world_can_be_made_windy(world):
    # The world starts calm and carries the system sim_weather_node drives the wind through.
    plugin = next(
        p for p in world.findall('plugin') if p.get('filename') == 'gz-sim-wind-effects-system')
    assert float(plugin.find('force_approximation_scaling_factor').text) > 0.0
    assert floats(world.find('wind/linear_velocity').text) == [0.0, 0.0, 0.0]


def test_every_building_collides_exactly_where_the_planner_thinks(city, world):
    by_name = models(world)
    for b in city['buildings']:
        model = by_name[b['name']]
        pose = floats(model.find('pose').text)[:3]
        size = floats(model.find('link/collision/geometry/box/size').text)
        for axis in range(3):
            assert pose[axis] == pytest.approx((b['min'][axis] + b['max'][axis]) / 2, abs=1e-3)
            assert size[axis] == pytest.approx(b['max'][axis] - b['min'][axis], abs=1e-3)


def test_every_cylinder_collides_exactly_where_the_planner_thinks(city, world):
    by_name = models(world)
    for c in city['cylinders']:
        model = by_name[c['name']]
        assert floats(model.find('pose').text)[:3] == pytest.approx(c['base'], abs=1e-3)
        geom = model.find('link/collision/geometry/cylinder')
        assert float(geom.find('radius').text) == pytest.approx(c['radius'])
        assert float(geom.find('length').text) == pytest.approx(c['height'])


def test_no_fly_zones_are_visible_but_not_solid(city, world):
    by_name = models(world)
    for z in city['no_fly_zones']:
        link = by_name[z['name']].find('link')
        assert link.find('visual') is not None
        assert link.find('collision') is None


def test_every_depot_location_and_traffic_route_has_a_model(city, world):
    names = set(models(world))
    for depot in city['depots']:
        assert f'depot_{depot}' in names
    for location in city['locations']:
        assert f'dropoff_{location}' in names
    for route in city.get('traffic', []):
        model = models(world)[f"traffic_{route['name']}"]
        assert model.find('static').text == 'true'


def test_vehicles_are_spawned_on_the_surface_the_world_has_there(city, world):
    # A depot or location given as [x, y] is wherever the map's surface is. The launch file
    # (city_map.position) and the world generator have to agree on that height, or a vehicle
    # whose depot is on a roof or a rise is spawned inside it.
    by_name = models(world)
    for kind, entries in (('depot', city['depots']), ('dropoff', city['locations'])):
        for name, where in entries.items():
            pad = floats(by_name[f'{kind}_{name}'].find('pose').text)[:3]
            assert position(city, where) == pytest.approx(pad, abs=1e-3), name
    assert position(city, city['locations']['rooftop_cafe'])[2] == pytest.approx(18.0)

    hill = {'geofence': {'min': [-50, -50, 0], 'max': [50, 50, 100]},
            'buildings': [{'name': 'rise', 'min': [-10, -10, 0], 'max': [10, 10, 7.5]}],
            'cylinders': [{'name': 'mast', 'base': [30, 0, 0], 'radius': 1.0, 'height': 20.0}]}
    assert position(hill, [0.0, 0.0]) == (0.0, 0.0, 7.5)       # on the rise, not inside it
    assert position(hill, [0.0, 0.0, 2.0]) == (0.0, 0.0, 2.0)  # an explicit height stands
    assert surface_height(hill, 30.5, 0.0) == 20.0
    assert surface_height(hill, 40.0, 40.0) == 0.0


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz not installed')
def test_world_is_valid_sdf():
    result = subprocess.run(['gz', 'sdf', '-k', WORLD], capture_output=True, text=True)
    assert result.returncode == 0 and 'Valid' in result.stdout, result.stdout + result.stderr


@pytest.mark.parametrize('key', sorted(VEHICLES))
def test_vehicle_and_parcel_sdf(key):
    sim = VEHICLES[key]
    vehicle = ET.fromstring(vehicle_sdf(sim, 'v', 'p'))
    model = vehicle.find('model')
    assert model.find('include').get('merge') == 'true'
    assert model.find('include/uri').text == f'model://{sim.base_model}'
    joint = model.find('plugin')
    assert joint.get('name') == 'gz::sim::systems::DetachableJoint'
    assert joint.find('child_model').text == 'p'
    assert joint.find('parent_link').text == 'base_link'

    # Only an airframe without aerodynamics of its own gets Gazebo's wind drag; on a plane it
    # would be a second, made-up drag on top of the real one.
    has_wings = sim.base_model in ('standard_vtol', 'rc_cessna')
    assert (model.find('enable_wind') is not None) == (not has_wings)
    if not has_wings:
        assert model.find('enable_wind').text == 'true'

    parcel = ET.fromstring(parcel_sdf(sim, 'p')).find('model/link')
    assert float(parcel.find('inertial/mass').text) == pytest.approx(sim.parcel_mass)
    for axis in ('ixx', 'iyy', 'izz'):
        assert float(parcel.find(f'inertial/inertia/{axis}').text) > 0.0


@pytest.mark.parametrize('key', sorted(VEHICLES))
def test_parcel_starts_above_the_ground(key):
    sim = VEHICLES[key]
    x, y, z = parcel_spawn_position(sim, (0.0, 0.0, 0.0))
    assert z - sim.parcel_size[2] / 2 > 0.0


def test_x500_parcel_fits_between_the_legs():
    # x500_base geometry (PX4 Tools/simulation/gz/models/x500_base): belly collision bottom
    # at -0.018 from base_link; legs lean in to |y| = 0.071 at 7 cm below base_link.
    sim = VEHICLES['x500']
    top = sim.parcel_offset[2] + sim.parcel_size[2] / 2
    assert top < -0.018 - 0.02
    assert top <= -0.07 + 1e-9
    assert sim.parcel_size[1] / 2 < 0.071 - 0.01
