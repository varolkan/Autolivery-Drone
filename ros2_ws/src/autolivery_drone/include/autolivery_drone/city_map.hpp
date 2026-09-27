// Loads a city map YAML (see maps/urban_delivery.yaml) into the core WorldMap plus the named
// places the mission layer needs: depots and dropoff locations.

#ifndef AUTOLIVERY_DRONE__CITY_MAP_HPP_
#define AUTOLIVERY_DRONE__CITY_MAP_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_drone
{

struct GeoOrigin
{
  double latitude{0.0};
  double longitude{0.0};
  double altitude{0.0};
};

struct TrafficRoute
{
  std::string name;
  double radius{1.0};
  double speed{3.0};
  std::vector<autolivery_core::Vec3> waypoints;
};

// A straight-in landing for fixed-wings: loiter down over `approach` to its altitude, then fly
// the line to `touchdown` along `direction` (unit, horizontal, world frame).
struct Runway
{
  std::string name;
  autolivery_core::Vec3 touchdown;   // z = surface height
  autolivery_core::Vec3 approach;    // loiter-down point, z = approach altitude (world)
  autolivery_core::Vec3 direction;   // landing roll direction
  double loiter_radius{60.0};
  double glide_slope_deg{0.0};
};

struct CityMap
{
  std::string name;
  GeoOrigin origin;
  std::shared_ptr<autolivery_core::WorldMap> world;
  std::map<std::string, autolivery_core::Vec3> depots;
  std::map<std::string, autolivery_core::Vec3> locations;  // z = surface height
  std::map<std::string, Runway> runways;  // keyed by depot name
  std::vector<TrafficRoute> traffic;
};

// Parses and validates the file. Throws std::runtime_error naming the file and the problem;
// a map that loads is internally consistent (depots and locations inside the geofence and
// not inside obstacles).
CityMap loadCityMap(const std::string & path);

// Geodetic <-> world conversion around the map origin (local tangent plane; accurate to well
// under a metre within the few-kilometre extent of a delivery area).
autolivery_core::Vec3 geodeticToWorld(
  const GeoOrigin & origin, double latitude, double longitude, double altitude);

// The inverse: {latitude, longitude, altitude} of a world point.
autolivery_core::Vec3 worldToGeodetic(const GeoOrigin & origin, const autolivery_core::Vec3 & p);

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__CITY_MAP_HPP_
