// The prior map a delivery flight is planned against: the geofence, the buildings and other
// fixed structures, and no-fly zones.
//
// Obstacles are deliberately simple shapes (axis-aligned boxes and vertical cylinders). City
// blocks, towers, trees, masts and restricted airspace are all well approximated by them, and
// simple shapes let every query here be exact instead of sampled -- a sampled segment check
// with a 1 m step will happily fly through a 0.5 m lamp post.
//
// All clearance-based queries grow the obstacles by the clearance (Minkowski sum, rounded up
// to a box/cylinder), so "clear" always means "at least `clearance` metres from anything".

#ifndef AUTOLIVERY_CORE__WORLD_MAP_HPP_
#define AUTOLIVERY_CORE__WORLD_MAP_HPP_

#include <string>
#include <vector>

#include "autolivery_core/geometry.hpp"

namespace autolivery_core
{

struct Box
{
  std::string name;
  Vec3 min;
  Vec3 max;
};

struct Cylinder
{
  std::string name;
  Vec3 base;  // centre of the bottom face
  double radius{0.0};
  double height{0.0};
};

class WorldMap
{
public:
  WorldMap() = default;
  WorldMap(const Vec3 & bounds_min, const Vec3 & bounds_max);

  void addBuilding(const Box & box);
  void addCylinder(const Cylinder & cylinder);
  void addNoFlyZone(const Box & zone);

  const Vec3 & boundsMin() const {return bounds_min_;}
  const Vec3 & boundsMax() const {return bounds_max_;}
  const std::vector<Box> & buildings() const {return buildings_;}
  const std::vector<Cylinder> & cylinders() const {return cylinders_;}
  const std::vector<Box> & noFlyZones() const {return no_fly_zones_;}

  // Inside the geofence. The geofence is a hard limit and is never grown by clearance.
  bool inBounds(const Vec3 & p) const;

  // Out of bounds, or within `clearance` of any obstacle or no-fly zone.
  bool isBlocked(const Vec3 & p, double clearance) const;

  // Exact test of the straight segment a->b against every obstacle grown by `clearance`.
  bool segmentClear(const Vec3 & a, const Vec3 & b, double clearance) const;

  // Name of the first obstacle the segment hits ("geofence" if it leaves the bounds), or an
  // empty string if the segment is clear. Used to make planner failures explainable.
  std::string firstBlocker(const Vec3 & a, const Vec3 & b, double clearance) const;

  // Euclidean distance from p to the nearest obstacle surface (0 inside one). No-fly zones
  // count. Returns +inf on an empty map.
  double distanceToObstacles(const Vec3 & p) const;

  // Height of the highest structure under (x, y), or the ground (bounds_min.z) if none.
  // This is what a dropoff on a rooftop is measured from.
  double groundHeight(double x, double y) const;

  // Human-readable problems with the map itself (inverted boxes, obstacles outside the
  // geofence, ...). An empty result means the map is usable.
  std::vector<std::string> validate() const;

private:
  Vec3 bounds_min_{-1000.0, -1000.0, 0.0};
  Vec3 bounds_max_{1000.0, 1000.0, 120.0};
  std::vector<Box> buildings_;
  std::vector<Cylinder> cylinders_;
  std::vector<Box> no_fly_zones_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__WORLD_MAP_HPP_
