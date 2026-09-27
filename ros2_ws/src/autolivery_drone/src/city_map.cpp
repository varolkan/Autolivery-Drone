#include "autolivery_drone/city_map.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace autolivery_drone
{

using autolivery_core::Box;
using autolivery_core::Cylinder;
using autolivery_core::Vec3;
using autolivery_core::WorldMap;

namespace
{

constexpr double kDeg = autolivery_core::kPi / 180.0;
constexpr double kEarthRadius = 6378137.0;

class MapError : public std::runtime_error
{
public:
  MapError(const std::string & path, const std::string & what)
  : std::runtime_error("city map " + path + ": " + what) {}
};

Vec3 readVec(
  const YAML::Node & node, const std::string & path, const std::string & key,
  bool allow_2d = false)
{
  if (!node || !node.IsSequence() || (node.size() != 3 && !(allow_2d && node.size() == 2))) {
    throw MapError(path, key + " must be [x, y, z]" + (allow_2d ? " or [x, y]" : ""));
  }
  const Vec3 v{
    node[0].as<double>(), node[1].as<double>(), node.size() == 3 ? node[2].as<double>() : NAN};
  if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
    throw MapError(path, key + " is not finite");
  }
  return v;
}

double readDouble(const YAML::Node & node, const std::string & path, const std::string & key)
{
  if (!node || !node.IsScalar()) {
    throw MapError(path, key + " is missing");
  }
  return node.as<double>();
}

std::string readName(const YAML::Node & node, const std::string & path, const std::string & where)
{
  if (!node["name"] || !node["name"].IsScalar()) {
    throw MapError(path, where + " needs a name");
  }
  return node["name"].as<std::string>();
}

}  // namespace

CityMap loadCityMap(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw MapError(path, std::string("cannot parse: ") + e.what());
  }

  try {
    CityMap city;
    city.name = root["name"] ? root["name"].as<std::string>() : std::string("unnamed");

    if (const auto origin = root["origin"]) {
      city.origin.latitude = readDouble(origin["latitude"], path, "origin.latitude");
      city.origin.longitude = readDouble(origin["longitude"], path, "origin.longitude");
      city.origin.altitude = readDouble(origin["altitude"], path, "origin.altitude");
    }

    if (!root["geofence"]) {
      throw MapError(path, "geofence is required");
    }
    const Vec3 lo = readVec(root["geofence"]["min"], path, "geofence.min");
    const Vec3 hi = readVec(root["geofence"]["max"], path, "geofence.max");
    city.world = std::make_shared<WorldMap>(lo, hi);

    for (const auto & b : root["buildings"]) {
      const std::string name = readName(b, path, "building");
      city.world->addBuilding(
        {name, readVec(b["min"], path, name + ".min"), readVec(b["max"], path, name + ".max")});
    }
    for (const auto & c : root["cylinders"]) {
      const std::string name = readName(c, path, "cylinder");
      city.world->addCylinder(
        {name, readVec(c["base"], path, name + ".base"),
          readDouble(c["radius"], path, name + ".radius"),
          readDouble(c["height"], path, name + ".height")});
    }
    for (const auto & z : root["no_fly_zones"]) {
      const std::string name = readName(z, path, "no-fly zone");
      city.world->addNoFlyZone(
        {name, readVec(z["min"], path, name + ".min"), readVec(z["max"], path, name + ".max")});
    }
    const auto problems = city.world->validate();
    if (!problems.empty()) {
      throw MapError(path, problems.front());
    }

    auto resolve = [&](const std::string & kind, const std::string & name, Vec3 p) {
      if (std::isnan(p.z)) {
        p.z = city.world->groundHeight(p.x, p.y);
      }
      if (!city.world->inBounds(p)) {
        throw MapError(path, kind + " '" + name + "' is outside the geofence");
      }
      // Standing on a surface is fine; being inside a building or no-fly zone is not.
      const Vec3 above = p + Vec3{0.0, 0.0, 0.5};
      if (city.world->isBlocked(above, 0.0)) {
        throw MapError(path, kind + " '" + name + "' is inside an obstacle or no-fly zone");
      }
      return p;
    };

    if (!root["depots"] || root["depots"].size() == 0) {
      throw MapError(path, "at least one depot is required");
    }
    for (const auto & d : root["depots"]) {
      const auto name = d.first.as<std::string>();
      city.depots[name] = resolve("depot", name, readVec(d.second, path, "depots." + name, true));
    }
    for (const auto & l : root["locations"]) {
      const auto name = l.first.as<std::string>();
      city.locations[name] =
        resolve("location", name, readVec(l.second, path, "locations." + name, true));
    }

    for (const auto & r : root["runways"]) {
      const auto name = r.first.as<std::string>();
      const auto & n = r.second;
      if (!city.depots.count(name)) {
        throw MapError(path, "runway '" + name + "' has no depot of the same name");
      }
      Runway rw;
      rw.name = name;
      rw.touchdown = resolve(
        "runway touchdown", name, readVec(n["touchdown"], path, name + ".touchdown", true));
      const double heading =
        readDouble(n["landing_heading"], path, name + ".landing_heading") * kDeg;
      rw.direction = {std::sin(heading), std::cos(heading), 0.0};  // compass -> ENU
      const double distance = readDouble(n["approach_distance"], path, name + ".approach_distance");
      const double altitude = readDouble(n["approach_altitude"], path, name + ".approach_altitude");
      rw.loiter_radius = n["loiter_radius"] ? n["loiter_radius"].as<double>() : 60.0;
      if (!(distance > rw.loiter_radius) || !(altitude > 0.0)) {
        throw MapError(
                path,
                "runway '" + name + "' needs approach_distance > loiter_radius and a positive "
                "approach_altitude");
      }
      const Vec3 depot = city.depots.at(name);
      rw.approach = rw.touchdown - rw.direction * distance;
      rw.approach.z = depot.z + altitude;
      rw.glide_slope_deg = std::atan2(altitude, distance) / kDeg;
      // The whole approach, loiter circle included, has to be clear airspace.
      const Vec3 floor{rw.touchdown.x, rw.touchdown.y, rw.touchdown.z + 2.0};
      const std::string blocker = city.world->firstBlocker(rw.approach, floor, 5.0);
      if (!blocker.empty() || city.world->isBlocked(rw.approach, rw.loiter_radius)) {
        throw MapError(
                path, "runway '" + name + "' approach is obstructed" +
                (blocker.empty() ? std::string(" around the loiter circle") : " by " + blocker));
      }
      city.runways[name] = rw;
    }

    for (const auto & t : root["traffic"]) {
      TrafficRoute route;
      route.name = readName(t, path, "traffic");
      route.radius = t["radius"] ? t["radius"].as<double>() : 1.0;
      route.speed = t["speed"] ? t["speed"].as<double>() : 3.0;
      for (const auto & w : t["waypoints"]) {
        route.waypoints.push_back(readVec(w, path, route.name + ".waypoints"));
      }
      if (route.waypoints.size() < 2 || !(route.speed > 0.0)) {
        throw MapError(path, "traffic '" + route.name + "' needs 2+ waypoints and a speed");
      }
      city.traffic.push_back(route);
    }
    return city;
  } catch (const YAML::Exception & e) {
    throw MapError(path, e.what());
  }
}

Vec3 geodeticToWorld(const GeoOrigin & origin, double latitude, double longitude, double altitude)
{
  const double north = (latitude - origin.latitude) * kDeg * kEarthRadius;
  const double east = (longitude - origin.longitude) * kDeg * kEarthRadius *
    std::cos(origin.latitude * kDeg);
  return {east, north, altitude - origin.altitude};
}

Vec3 worldToGeodetic(const GeoOrigin & origin, const Vec3 & p)
{
  const double latitude = origin.latitude + p.y / kEarthRadius / kDeg;
  const double longitude =
    origin.longitude + p.x / (kEarthRadius * std::cos(origin.latitude * kDeg)) / kDeg;
  return {latitude, longitude, origin.altitude + p.z};
}

}  // namespace autolivery_drone
