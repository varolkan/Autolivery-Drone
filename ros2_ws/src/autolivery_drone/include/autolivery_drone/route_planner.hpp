// The global planner the nodes use: A* or RRT* over the city map, followed by shortcut
// smoothing and optional densification. Builds the voxel grid once, up front.

#ifndef AUTOLIVERY_DRONE__ROUTE_PLANNER_HPP_
#define AUTOLIVERY_DRONE__ROUTE_PLANNER_HPP_

#include <memory>
#include <string>

#include "autolivery_core/astar.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_core/rrt_star.hpp"
#include "autolivery_core/voxel_grid.hpp"
#include "autolivery_core/world_map.hpp"
#include "autolivery_drone/params.hpp"

namespace autolivery_drone
{

class RoutePlanner
{
public:
  RoutePlanner(
    std::shared_ptr<const autolivery_core::WorldMap> map, double clearance,
    PlannerSettings settings);

  // `algorithm` overrides the configured default when non-empty.
  autolivery_core::PlanResult plan(
    const autolivery_core::Vec3 & from, const autolivery_core::Vec3 & to,
    const std::string & algorithm = {}) const;

  autolivery_core::PlanFn asPlanFn() const;

  const PlannerSettings & settings() const {return settings_;}
  const autolivery_core::VoxelGrid & grid() const {return *grid_;}

private:
  std::shared_ptr<const autolivery_core::WorldMap> map_;
  double clearance_;
  PlannerSettings settings_;
  std::shared_ptr<const autolivery_core::VoxelGrid> grid_;
  std::unique_ptr<autolivery_core::AStarPlanner> astar_;
  std::unique_ptr<autolivery_core::RrtStarPlanner> rrt_;
};

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__ROUTE_PLANNER_HPP_
