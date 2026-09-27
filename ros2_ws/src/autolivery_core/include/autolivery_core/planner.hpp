// Common interface for the global path planners (A* and RRT*).
//
// Planners never throw for "no path" -- that is an expected outcome in a city (a dropoff
// inside a no-fly zone, a geofence that is too tight) and the mission has to handle it. They
// return a result with success=false and a message a human can act on.

#ifndef AUTOLIVERY_CORE__PLANNER_HPP_
#define AUTOLIVERY_CORE__PLANNER_HPP_

#include <cstddef>
#include <string>

#include "autolivery_core/geometry.hpp"

namespace autolivery_core
{

struct PlanResult
{
  bool success{false};
  Path path;
  std::string message;
  std::size_t iterations{0};  // A*: nodes expanded, RRT*: samples drawn
  double planning_time{0.0};  // seconds
};

class PathPlanner
{
public:
  virtual ~PathPlanner() = default;
  virtual PlanResult plan(const Vec3 & start, const Vec3 & goal) const = 0;
  virtual std::string name() const = 0;
};

// Edge cost used by both planners: Euclidean distance with the vertical component weighted
// by `climb_cost`. Climbing costs a multirotor far more energy than cruising, and flying at
// the altitude the operator chose keeps the vehicle where the airspace was deconflicted, so
// both planners prefer to route around a tower rather than over it unless over is much
// shorter.
inline double weightedDistance(const Vec3 & a, const Vec3 & b, double climb_cost)
{
  const Vec3 d = b - a;
  return std::sqrt(d.x * d.x + d.y * d.y + climb_cost * climb_cost * d.z * d.z);
}

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__PLANNER_HPP_
