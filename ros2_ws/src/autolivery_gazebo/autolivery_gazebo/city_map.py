"""The little the simulation has to read out of a city map itself.

A depot or a location may be given as [x, y]: it then sits on whatever the map has there, the
ground or a roof. The flight stack (autolivery_drone's map loader) and generate_world.py both
resolve it that way, so the launch file has to as well, or a vehicle whose depot is on a roof
or a rise would be spawned inside it.
"""

import math


def surface_height(city, x, y):
    """Top of the highest structure in the map under (x, y), or the ground if there is none."""
    height = city['geofence']['min'][2]
    for b in city.get('buildings', []):
        if b['min'][0] <= x <= b['max'][0] and b['min'][1] <= y <= b['max'][1]:
            height = max(height, b['max'][2])
    for c in city.get('cylinders', []):
        if math.hypot(x - c['base'][0], y - c['base'][1]) <= c['radius']:
            height = max(height, c['base'][2] + c['height'])
    return height


def position(city, where):
    """(x, y, z) of a depot or location entry; [x, y] entries get the surface height."""
    if len(where) == 3:
        return (float(where[0]), float(where[1]), float(where[2]))
    return (float(where[0]), float(where[1]), float(surface_height(city, where[0], where[1])))
