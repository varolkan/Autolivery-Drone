// 26-connected A* over a VoxelGrid.
//
// Deterministic and complete at the grid resolution, which makes it the default for
// deliveries: the same request always produces the same route, and "no path" really means
// there is none at this resolution and clearance.

#ifndef AUTOLIVERY_CORE__ASTAR_HPP_
#define AUTOLIVERY_CORE__ASTAR_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "autolivery_core/planner.hpp"
#include "autolivery_core/voxel_grid.hpp"

namespace autolivery_core
{

struct AStarParams
{
  double climb_cost{1.5};              // vertical distance multiplier, see weightedDistance()
  std::size_t max_expansions{3'000'000};
  int snap_radius{3};                  // voxels searched for a free cell near start/goal
};

class AStarPlanner : public PathPlanner
{
public:
  explicit AStarPlanner(std::shared_ptr<const VoxelGrid> grid, AStarParams params = {});

  PlanResult plan(const Vec3 & start, const Vec3 & goal) const override;
  std::string name() const override {return "astar";}

private:
  std::shared_ptr<const VoxelGrid> grid_;
  AStarParams params_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__ASTAR_HPP_
