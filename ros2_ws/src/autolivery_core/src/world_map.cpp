#include "autolivery_core/world_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace autolivery_core
{

namespace
{

bool pointInBox(const Vec3 & p, const Box & b, double c)
{
  return p.x >= b.min.x - c && p.x <= b.max.x + c &&
         p.y >= b.min.y - c && p.y <= b.max.y + c &&
         p.z >= b.min.z - c && p.z <= b.max.z + c;
}

bool pointInCylinder(const Vec3 & p, const Cylinder & cyl, double c)
{
  if (p.z < cyl.base.z - c || p.z > cyl.base.z + cyl.height + c) {
    return false;
  }
  const double r = cyl.radius + c;
  const double dx = p.x - cyl.base.x;
  const double dy = p.y - cyl.base.y;
  return dx * dx + dy * dy <= r * r;
}

// Slab test: does segment a->b intersect the box grown by c?
bool segmentHitsBox(const Vec3 & a, const Vec3 & b, const Box & box, double c)
{
  const double lo[3] = {box.min.x - c, box.min.y - c, box.min.z - c};
  const double hi[3] = {box.max.x + c, box.max.y + c, box.max.z + c};
  const double origin[3] = {a.x, a.y, a.z};
  const double dir[3] = {b.x - a.x, b.y - a.y, b.z - a.z};

  double t_enter = 0.0;
  double t_exit = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < 1e-12) {
      if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) {
        return false;
      }
      continue;
    }
    double t0 = (lo[axis] - origin[axis]) / dir[axis];
    double t1 = (hi[axis] - origin[axis]) / dir[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_enter = std::max(t_enter, t0);
    t_exit = std::min(t_exit, t1);
    if (t_enter > t_exit) {
      return false;
    }
  }
  return true;
}

// Segment against a vertical cylinder grown by c radially and vertically: clip the segment to
// the cylinder's z-slab, then find the closest horizontal approach within that sub-segment.
bool segmentHitsCylinder(const Vec3 & a, const Vec3 & b, const Cylinder & cyl, double c)
{
  const double z_lo = cyl.base.z - c;
  const double z_hi = cyl.base.z + cyl.height + c;
  const Vec3 d = b - a;

  double t0 = 0.0;
  double t1 = 1.0;
  if (std::abs(d.z) < 1e-12) {
    if (a.z < z_lo || a.z > z_hi) {
      return false;
    }
  } else {
    double ta = (z_lo - a.z) / d.z;
    double tb = (z_hi - a.z) / d.z;
    if (ta > tb) {
      std::swap(ta, tb);
    }
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    if (t0 > t1) {
      return false;
    }
  }

  const double ox = a.x - cyl.base.x;
  const double oy = a.y - cyl.base.y;
  const double dd = d.x * d.x + d.y * d.y;
  double t = t0;
  if (dd > 1e-12) {
    t = std::clamp(-(ox * d.x + oy * d.y) / dd, t0, t1);
  }
  const double px = ox + d.x * t;
  const double py = oy + d.y * t;
  const double r = cyl.radius + c;
  return px * px + py * py <= r * r;
}

double distanceToBox(const Vec3 & p, const Box & b)
{
  const double dx = std::max({b.min.x - p.x, 0.0, p.x - b.max.x});
  const double dy = std::max({b.min.y - p.y, 0.0, p.y - b.max.y});
  const double dz = std::max({b.min.z - p.z, 0.0, p.z - b.max.z});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double distanceToCylinder(const Vec3 & p, const Cylinder & cyl)
{
  const double radial = std::max(0.0, std::hypot(p.x - cyl.base.x, p.y - cyl.base.y) - cyl.radius);
  const double vertical =
    std::max({cyl.base.z - p.z, 0.0, p.z - (cyl.base.z + cyl.height)});
  return std::hypot(radial, vertical);
}

std::string describe(const Vec3 & v)
{
  std::ostringstream out;
  out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return out.str();
}

}  // namespace

WorldMap::WorldMap(const Vec3 & bounds_min, const Vec3 & bounds_max)
: bounds_min_(bounds_min), bounds_max_(bounds_max)
{
}

void WorldMap::addBuilding(const Box & box) {buildings_.push_back(box);}
void WorldMap::addCylinder(const Cylinder & cylinder) {cylinders_.push_back(cylinder);}
void WorldMap::addNoFlyZone(const Box & zone) {no_fly_zones_.push_back(zone);}

bool WorldMap::inBounds(const Vec3 & p) const
{
  return p.x >= bounds_min_.x && p.x <= bounds_max_.x &&
         p.y >= bounds_min_.y && p.y <= bounds_max_.y &&
         p.z >= bounds_min_.z && p.z <= bounds_max_.z;
}

bool WorldMap::isBlocked(const Vec3 & p, double clearance) const
{
  if (!inBounds(p)) {
    return true;
  }
  for (const auto & b : buildings_) {
    if (pointInBox(p, b, clearance)) {
      return true;
    }
  }
  for (const auto & cyl : cylinders_) {
    if (pointInCylinder(p, cyl, clearance)) {
      return true;
    }
  }
  for (const auto & z : no_fly_zones_) {
    if (pointInBox(p, z, clearance)) {
      return true;
    }
  }
  return false;
}

bool WorldMap::segmentClear(const Vec3 & a, const Vec3 & b, double clearance) const
{
  return firstBlocker(a, b, clearance).empty();
}

std::string WorldMap::firstBlocker(const Vec3 & a, const Vec3 & b, double clearance) const
{
  // The geofence is convex, so a segment is inside it iff both endpoints are.
  if (!inBounds(a) || !inBounds(b)) {
    return "geofence";
  }
  for (const auto & box : buildings_) {
    if (segmentHitsBox(a, b, box, clearance)) {
      return box.name.empty() ? "building" : box.name;
    }
  }
  for (const auto & cyl : cylinders_) {
    if (segmentHitsCylinder(a, b, cyl, clearance)) {
      return cyl.name.empty() ? "structure" : cyl.name;
    }
  }
  for (const auto & zone : no_fly_zones_) {
    if (segmentHitsBox(a, b, zone, clearance)) {
      return zone.name.empty() ? "no-fly zone" : zone.name;
    }
  }
  return {};
}

double WorldMap::distanceToObstacles(const Vec3 & p) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & b : buildings_) {
    best = std::min(best, distanceToBox(p, b));
  }
  for (const auto & cyl : cylinders_) {
    best = std::min(best, distanceToCylinder(p, cyl));
  }
  for (const auto & z : no_fly_zones_) {
    best = std::min(best, distanceToBox(p, z));
  }
  return best;
}

double WorldMap::groundHeight(double x, double y) const
{
  double ground = bounds_min_.z;
  for (const auto & b : buildings_) {
    if (x >= b.min.x && x <= b.max.x && y >= b.min.y && y <= b.max.y) {
      ground = std::max(ground, b.max.z);
    }
  }
  for (const auto & cyl : cylinders_) {
    if (std::hypot(x - cyl.base.x, y - cyl.base.y) <= cyl.radius) {
      ground = std::max(ground, cyl.base.z + cyl.height);
    }
  }
  return ground;
}

std::vector<std::string> WorldMap::validate() const
{
  std::vector<std::string> problems;
  if (bounds_min_.x >= bounds_max_.x || bounds_min_.y >= bounds_max_.y ||
    bounds_min_.z >= bounds_max_.z)
  {
    problems.push_back(
      "geofence is empty: min " + describe(bounds_min_) + " max " + describe(bounds_max_));
  }
  auto check_box = [&](const Box & b, const char * kind) {
      if (b.min.x >= b.max.x || b.min.y >= b.max.y || b.min.z >= b.max.z) {
        problems.push_back(
        std::string(kind) + " '" + b.name + "' has min " + describe(b.min) + " not below max " +
        describe(b.max));
      }
      if (!b.min.isFinite() || !b.max.isFinite()) {
        problems.push_back(std::string(kind) + " '" + b.name + "' has non-finite corners");
      }
    };
  for (const auto & b : buildings_) {
    check_box(b, "building");
  }
  for (const auto & z : no_fly_zones_) {
    check_box(z, "no-fly zone");
  }
  for (const auto & cyl : cylinders_) {
    if (!(cyl.radius > 0.0) || !(cyl.height > 0.0) || !cyl.base.isFinite()) {
      problems.push_back("cylinder '" + cyl.name + "' needs positive radius and height");
    }
  }
  return problems;
}

}  // namespace autolivery_core
