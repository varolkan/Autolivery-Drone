// RRT* in continuous 3D space, checked exactly against the WorldMap.
//
// Useful where the voxel grid is too coarse (narrow gaps between towers) or too large (long
// rural legs). It is an anytime planner: it keeps improving the best path until it runs out
// of iterations or time, so `max_planning_time` bounds how long the mission loop can block.

#ifndef AUTOLIVERY_CORE__RRT_STAR_HPP_
#define AUTOLIVERY_CORE__RRT_STAR_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "autolivery_core/planner.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

struct RrtStarParams
{
  std::size_t max_iterations{4000};
  double max_planning_time{1.0};   // seconds
  double step_size{10.0};          // max edge length when extending the tree
  double goal_bias{0.1};           // probability of sampling the goal directly
  double rewire_radius{25.0};
  double clearance{3.0};
  double climb_cost{1.5};
  // Sampling band. Restricting samples to the altitudes a delivery actually uses makes the
  // tree converge an order of magnitude faster than sampling the whole geofence.
  double min_altitude{-std::numeric_limits<double>::infinity()};
  double max_altitude{std::numeric_limits<double>::infinity()};
  std::uint32_t seed{0};           // 0 = seed from std::random_device
};

class RrtStarPlanner : public PathPlanner
{
public:
  explicit RrtStarPlanner(std::shared_ptr<const WorldMap> map, RrtStarParams params = {});

  PlanResult plan(const Vec3 & start, const Vec3 & goal) const override;
  std::string name() const override {return "rrt_star";}

private:
  std::shared_ptr<const WorldMap> map_;
  RrtStarParams params_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__RRT_STAR_HPP_
