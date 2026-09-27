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
    r = rrt_->plan(from, to);
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

autolivery_core::PlanFn RoutePlanner::asPlanFn() const
{
  return [this](const Vec3 & from, const Vec3 & to) {return plan(from, to);};
}

}  // namespace autolivery_drone
