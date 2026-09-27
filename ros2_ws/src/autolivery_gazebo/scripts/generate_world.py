#!/usr/bin/env python3
"""Build a Gazebo world from a city map YAML.

    generate_world.py <map.yaml> <world.sdf>          write the world
    generate_world.py --check <map.yaml> <world.sdf>  exit 1 if the world is out of date

The city map (autolivery_drone/maps/*.yaml) is the single source of truth: the planner flies
against it, and this script turns the same boxes and cylinders into Gazebo models, so what
the vehicle collides with in simulation is exactly what it planned around. Output is
deterministic, which is what lets the package test check the committed world is current.

The world has no vehicles. sim.launch.py spawns the vehicle and its parcel at the depot.
"""

import argparse
import math
import sys

import yaml

# World plugins PX4's gz_bridge needs (IMU, baro, airspeed, GPS, mag) plus the usual
# physics/commands/scene systems. Kept explicit so the world does not depend on PX4's
# server.config being on GZ_SIM_SERVER_CONFIG_PATH.
PLUGINS = [
    ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ('gz-sim-contact-system', 'gz::sim::systems::Contact'),
    ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
    ('gz-sim-air-pressure-system', 'gz::sim::systems::AirPressure'),
    ('gz-sim-air-speed-system', 'gz::sim::systems::AirSpeed'),
    ('gz-sim-apply-link-wrench-system', 'gz::sim::systems::ApplyLinkWrench'),
    ('gz-sim-navsat-system', 'gz::sim::systems::NavSat'),
    ('gz-sim-magnetometer-system', 'gz::sim::systems::Magnetometer'),
]

# Facade colours, picked per building by index so the city is readable from above.
FACADES = [
    (0.66, 0.68, 0.72), (0.78, 0.74, 0.66), (0.55, 0.60, 0.68), (0.72, 0.62, 0.55),
    (0.62, 0.70, 0.74), (0.80, 0.78, 0.74), (0.58, 0.56, 0.60),
]


def f(v):
    """Format a number compactly and deterministically."""
    text = f'{v:.4f}'.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def vec(*values):
    return ' '.join(f(v) for v in values)


def material(r, g, b, a=1.0, emissive=None):
    lines = [
        '<material>',
        f'  <ambient>{vec(r, g, b, a)}</ambient>',
        f'  <diffuse>{vec(r, g, b, a)}</diffuse>',
        f'  <specular>{vec(0.1, 0.1, 0.1, a)}</specular>',
    ]
    if emissive:
        lines.append(f'  <emissive>{vec(*emissive, 1)}</emissive>')
    lines.append('</material>')
    return lines


def indent(lines, n):
    return [' ' * n + line for line in lines]


def static_model(name, pose, link_body):
    return [
        f'<model name="{name}">',
        '  <static>true</static>',
        f'  <pose>{vec(*pose)} 0 0 0</pose>',
        '  <link name="link">',
        *indent(link_body, 4),
        '  </link>',
        '</model>',
    ]


def box_geometry(size):
    return ['<geometry>', f'  <box><size>{vec(*size)}</size></box>', '</geometry>']


def cylinder_geometry(radius, length):
    return [
        '<geometry>',
        f'  <cylinder><radius>{f(radius)}</radius><length>{f(length)}</length></cylinder>',
        '</geometry>',
    ]


def building(index, b):
    lo, hi = b['min'], b['max']
    size = [hi[i] - lo[i] for i in range(3)]
    centre = [(hi[i] + lo[i]) / 2 for i in range(3)]
    colour = FACADES[index % len(FACADES)]
    roof = (0.35, 0.36, 0.38)
    body = [
        '<collision name="collision">',
        *indent(box_geometry(size), 2),
        '</collision>',
        '<visual name="facade">',
        *indent(box_geometry(size), 2),
        *indent(material(*colour), 2),
        '</visual>',
        # A thin darker roof slab reads as a roof from above and in the follow camera.
        '<visual name="roof">',
        f'  <pose>0 0 {f(size[2] / 2 + 0.05)} 0 0 0</pose>',
        *indent(box_geometry([size[0] - 0.4, size[1] - 0.4, 0.1]), 2),
        *indent(material(*roof), 2),
        '</visual>',
    ]
    return static_model(b['name'], centre, body)


def cylinder(c):
    base = c['base']
    r, h = c['radius'], c['height']
    body = [
        '<collision name="collision">',
        f'  <pose>0 0 {f(h / 2)} 0 0 0</pose>',
        *indent(cylinder_geometry(r, h), 2),
        '</collision>',
    ]
    if 'tree' in c['name']:
        trunk = 0.35 * h
        body += [
            '<visual name="trunk">',
            f'  <pose>0 0 {f(trunk / 2)} 0 0 0</pose>',
            *indent(cylinder_geometry(max(0.15, r * 0.12), trunk), 2),
            *indent(material(0.36, 0.25, 0.14), 2),
            '</visual>',
            '<visual name="canopy">',
            f'  <pose>0 0 {f(trunk + (h - trunk) / 2)} 0 0 0</pose>',
            *indent(cylinder_geometry(r, h - trunk), 2),
            *indent(material(0.18, 0.45, 0.16), 2),
            '</visual>',
        ]
    else:
        body += [
            '<visual name="visual">',
            f'  <pose>0 0 {f(h / 2)} 0 0 0</pose>',
            *indent(cylinder_geometry(r, h), 2),
            *indent(material(0.7, 0.7, 0.72), 2),
            '</visual>',
            # Obstruction light on top, as on any real mast.
            '<visual name="beacon">',
            f'  <pose>0 0 {f(h + 0.3)} 0 0 0</pose>',
            '  <geometry><sphere><radius>0.4</radius></sphere></geometry>',
            *indent(material(0.9, 0.1, 0.1, emissive=(0.8, 0.0, 0.0)), 2),
            '</visual>',
        ]
    return static_model(c['name'], base, body)


def no_fly_zone(z):
    lo, hi = z['min'], z['max']
    size = [hi[i] - lo[i] for i in range(3)]
    centre = [(hi[i] + lo[i]) / 2 for i in range(3)]
    # Visual only: restricted airspace is a rule, not a wall. The planner and PX4 geofence
    # keep the vehicle out; physics should not pretend it would bounce off.
    body = [
        '<visual name="zone">',
        '  <cast_shadows>false</cast_shadows>',
        '  <transparency>0.85</transparency>',
        *indent(box_geometry(size), 2),
        *indent(material(0.9, 0.1, 0.1, 0.15), 2),
        '</visual>',
    ]
    return static_model(z['name'], centre, body)


def pad(name, position, colour, radius):
    body = [
        '<visual name="pad">',
        '  <pose>0 0 0.02 0 0 0</pose>',
        *indent(cylinder_geometry(radius, 0.04), 2),
        *indent(material(*colour), 2),
        '</visual>',
        '<visual name="mark">',
        '  <pose>0 0 0.045 0 0 0</pose>',
        *indent(cylinder_geometry(radius * 0.35, 0.01), 2),
        *indent(material(0.95, 0.95, 0.95), 2),
        '</visual>',
    ]
    return static_model(name, position, body)


def traffic_model(route):
    x, y, z = route['waypoints'][0]
    body = [
        '<visual name="body">',
        *indent(box_geometry([0.7, 0.7, 0.18]), 2),
        *indent(material(0.85, 0.2, 0.1), 2),
        '</visual>',
    ]
    for i, (dx, dy) in enumerate(((0.45, 0.45), (-0.45, 0.45), (0.45, -0.45), (-0.45, -0.45))):
        body += [
            f'<visual name="rotor_{i}">',
            f'  <pose>{vec(dx, dy, 0.12)} 0 0 0</pose>',
            *indent(cylinder_geometry(0.25, 0.02), 2),
            *indent(material(0.1, 0.1, 0.1, 0.8), 2),
            '</visual>',
        ]
    return static_model(f"traffic_{route['name']}", (x, y, z), body)


def ground(geofence):
    lo, hi = geofence['min'], geofence['max']
    margin = 200.0
    size = (hi[0] - lo[0] + 2 * margin, hi[1] - lo[1] + 2 * margin)
    centre = ((hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2)
    return [
        '<model name="ground_plane">',
        '  <static>true</static>',
        '  <link name="link">',
        '    <collision name="collision">',
        '      <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>',
        '    </collision>',
        '    <visual name="grass">',
        f'      <pose>{vec(centre[0], centre[1], 0)} 0 0 0</pose>',
        '      <geometry><plane><normal>0 0 1</normal>'
        f'<size>{vec(*size)}</size></plane></geometry>',
        *indent(material(0.36, 0.50, 0.30), 6),
        '    </visual>',
        '  </link>',
        '</model>',
    ]


def surface(name, lo, hi, colour, z=0.01):
    size = (hi[0] - lo[0], hi[1] - lo[1], 0.02)
    centre = ((hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, z)
    return static_model(name, centre, [
        '<visual name="surface">',
        *indent(box_geometry(size), 2),
        *indent(material(*colour), 2),
        '</visual>',
    ])


def generate(city):
    origin = city.get('origin', {})
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f"<!-- Generated by autolivery_gazebo/scripts/generate_world.py from the '{city['name']}'",
        '     city map. Do not edit by hand: change the map and regenerate. -->',
        '<sdf version="1.9">',
        f"  <world name=\"{city['name']}\">",
        '    <physics type="ode">',
        '      <max_step_size>0.004</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '      <real_time_update_rate>250</real_time_update_rate>',
        '    </physics>',
    ]
    for filename, name in PLUGINS:
        lines.append(f'    <plugin filename="{filename}" name="{name}"/>')
    lines += [
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        '      <render_engine>ogre2</render_engine>',
        '    </plugin>',
        '    <gravity>0 0 -9.8</gravity>',
        '    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>',
        '    <atmosphere type="adiabatic"/>',
        '    <scene>',
        '      <grid>false</grid>',
        '      <ambient>0.5 0.5 0.5 1</ambient>',
        '      <background>0.65 0.78 0.92 1</background>',
        '      <shadows>true</shadows>',
        '    </scene>',
        '    <light name="sun" type="directional">',
        '      <pose>0 0 500 0 0 0</pose>',
        '      <cast_shadows>true</cast_shadows>',
        '      <intensity>1</intensity>',
        '      <direction>-0.4 0.3 -0.85</direction>',
        '      <diffuse>0.95 0.95 0.9 1</diffuse>',
        '      <specular>0.3 0.3 0.3 1</specular>',
        '    </light>',
        '    <spherical_coordinates>',
        '      <surface_model>EARTH_WGS84</surface_model>',
        '      <world_frame_orientation>ENU</world_frame_orientation>',
        f"      <latitude_deg>{origin.get('latitude', 47.397971)}</latitude_deg>",
        f"      <longitude_deg>{origin.get('longitude', 8.546164)}</longitude_deg>",
        f"      <elevation>{origin.get('altitude', 0.0)}</elevation>",
        '    </spherical_coordinates>',
    ]
    models = []
    models.append(ground(city['geofence']))
    buildings = city.get('buildings', [])
    if buildings:
        xs = [b['min'][0] for b in buildings if b['min'][0] > -100] + \
             [b['max'][0] for b in buildings if b['min'][0] > -100]
        ys = [b['min'][1] for b in buildings if b['min'][0] > -100] + \
             [b['max'][1] for b in buildings if b['min'][0] > -100]
        if xs:
            models.append(surface(
                'streets', (min(xs) - 30, min(ys) - 30), (max(xs) + 30, max(ys) + 30),
                (0.32, 0.32, 0.34)))
    for name, where in sorted(city.get('depots', {}).items()):
        if name == 'airstrip':
            models.append(surface(
                'runway', (where[0] - 20, where[1] - 8), (where[0] + 300, where[1] + 8),
                (0.22, 0.22, 0.24), z=0.015))
    for i, b in enumerate(buildings):
        models.append(building(i, b))
    for c in city.get('cylinders', []):
        models.append(cylinder(c))
    for z in city.get('no_fly_zones', []):
        models.append(no_fly_zone(z))

    def surface_height(x, y):
        h = city['geofence']['min'][2]
        for b in buildings:
            if b['min'][0] <= x <= b['max'][0] and b['min'][1] <= y <= b['max'][1]:
                h = max(h, b['max'][2])
        for c in city.get('cylinders', []):
            if math.hypot(x - c['base'][0], y - c['base'][1]) <= c['radius']:
                h = max(h, c['base'][2] + c['height'])
        return h

    for name, where in sorted(city.get('depots', {}).items()):
        z = where[2] if len(where) == 3 else surface_height(where[0], where[1])
        models.append(pad(f'depot_{name}', (where[0], where[1], z), (0.15, 0.35, 0.85), 3.0))
    for name, where in sorted(city.get('locations', {}).items()):
        z = where[2] if len(where) == 3 else surface_height(where[0], where[1])
        models.append(pad(f'dropoff_{name}', (where[0], where[1], z), (0.95, 0.55, 0.05), 2.0))
    for route in city.get('traffic', []):
        models.append(traffic_model(route))

    for m in models:
        lines += indent(m, 4)
    lines += ['  </world>', '</sdf>', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true', help='verify instead of writing')
    parser.add_argument('map', help='city map YAML')
    parser.add_argument('world', help='world SDF to write or check')
    args = parser.parse_args(argv)

    with open(args.map) as fh:
        city = yaml.safe_load(fh)
    sdf = generate(city)
    if args.check:
        try:
            with open(args.world) as fh:
                current = fh.read()
        except FileNotFoundError:
            current = None
        if current != sdf:
            print(f'{args.world} is out of date with {args.map}; regenerate it:\n'
                  f'  {sys.argv[0]} {args.map} {args.world}', file=sys.stderr)
            return 1
        return 0
    with open(args.world, 'w') as fh:
        fh.write(sdf)
    print(f'wrote {args.world}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
