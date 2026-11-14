#include "autolivery_drone/route_planner.hpp"

#include <stdexcept>
#include <utility>

#include "autolivery_core/path_smoother.hpp"

namespace autolivery_drone
{

using autolivery_core::PlanResult;
using autolivery_core::Vec3;

RoutePlanner::RoutePlanner(
  std::shared_ptr<const autolivery_core::WorldMap> map, double clearance,
  PlannerSettings settings)
: map_(std::move(map)), clearance_(clearance), settings_(std::move(settings))
{
  if (!map_) {
    throw std::invalid_argument("RoutePlanner needs a map");
  }
  grid_ = std::make_shared<autolivery_core::VoxelGrid>(*map_, settings_.resolution, clearance_);
  astar_ = std::make_unique<autolivery_core::AStarPlanner>(grid_, settings_.astar);
  auto rrt = settings_.rrt;
  rrt.clearance = clearance_;
  rrt_ = std::make_unique<autolivery_core::RrtStarPlanner>(map_, rrt);
}

PlanResult RoutePlanner::plan(
  const Vec3 & from, const Vec3 & to,
  const std::string & algorithm) const
{
  const std::string which = algorithm.empty() ? settings_.algorithm : algorithm;
  PlanResult r;
  if (which == "astar") {
    r = astar_->plan(from, to);
  } else if (which == "rrt_star") {
    r = planRrtStar(from, to);
    if (!r.success && algorithm.empty()) {
      // RRT* is a sampling planner: it can come back empty-handed on a route that exists,
      // more often the less time it is given. When it is the configured planner the mission
      // needs a route, not an opinion about RRT*, and A* is complete at its resolution. (A
      // caller that asked for rrt_star by name gets rrt_star's answer.)
      PlanResult fallback = astar_->plan(from, to);
      if (fallback.success) {
        fallback.message += " (by astar; rrt_star: " + r.message + ")";
        fallback.planning_time += r.planning_time;
        r = std::move(fallback);
      }
    }
  } else {
    r.message = "unknown planner '" + which + "'";
    return r;
  }
  if (!r.success) {
    r.message = which + ": " + r.message;
    return r;
  }
  r.path = autolivery_core::shortcutPath(r.path, *map_, clearance_);
  r.path = autolivery_core::removeDuplicates(r.path, 0.5);
  if (settings_.max_segment > 0.0) {
    r.path = autolivery_core::densifyPath(r.path, settings_.max_segment);
  }
  return r;
}

PlanResult RoutePlanner::planRrtStar(const Vec3 & from, const Vec3 & to) const
{
  // RRT* cannot leave a start that is inside a clearance margin: every edge out of it fails
  // the clearance check. A vehicle can be there all the same (pushed aside by avoidance, or
  // asked to replan next to a wall), so do what A* does: plan from the nearest free voxel and
  // keep the real position as the first point of the route.
  if (map_->inBounds(from) && map_->isBlocked(from, clearance_)) {
    const auto free = grid_->nearestFree(grid_->toIndex(from), settings_.astar.snap_radius);
    if (free) {
      PlanResult r = rrt_->plan(grid_->toWorld(*free), to);
      if (r.success) {
        r.path.insert(r.path.begin(), from);
        r.message += " (start moved to nearest free cell)";
      }
      return r;
    }
  }
  return rrt_->plan(from, to);
}

autolivery_core::PlanFn RoutePlanner::asPlanFn() const
{
  return [this](const Vec3 & from, const Vec3 & to) {return plan(from, to);};
}

}  // namespace autolivery_drone
