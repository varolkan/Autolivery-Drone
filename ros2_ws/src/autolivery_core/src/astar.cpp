#include "autolivery_core/astar.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autolivery_core
{

namespace
{

struct Step
{
  int dx;
  int dy;
  int dz;
  double cost;
};

std::array<Step, 26> makeSteps(double resolution, double climb_cost)
{
  std::array<Step, 26> steps{};
  std::size_t n = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const double cost = resolution *
          std::sqrt(dx * dx + dy * dy + climb_cost * climb_cost * dz * dz);
        steps[n++] = {dx, dy, dz, cost};
      }
    }
  }
  return steps;
}

}  // namespace

AStarPlanner::AStarPlanner(std::shared_ptr<const VoxelGrid> grid, AStarParams params)
: grid_(std::move(grid)), params_(params)
{
  if (!grid_) {
    throw std::invalid_argument("AStarPlanner needs a voxel grid");
  }
  if (params_.climb_cost < 1.0) {
    // Below 1 the heuristic stops being admissible for the edge costs we use.
    throw std::invalid_argument("climb_cost must be >= 1");
  }
}

PlanResult AStarPlanner::plan(const Vec3 & start, const Vec3 & goal) const
{
  const auto t0 = std::chrono::steady_clock::now();
  PlanResult result;
  auto finish = [&](PlanResult & r) -> PlanResult {
      r.planning_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      return r;
    };

  const VoxelGrid & grid = *grid_;
  const GridIndex start_raw = grid.toIndex(start);
  const GridIndex goal_raw = grid.toIndex(goal);
  if (!grid.contains(start_raw)) {
    result.message = "start is outside the geofence";
    return finish(result);
  }
  if (!grid.contains(goal_raw)) {
    result.message = "goal is outside the geofence";
    return finish(result);
  }
  const auto start_idx = grid.nearestFree(start_raw, params_.snap_radius);
  if (!start_idx) {
    result.message = "start is inside an obstacle or its clearance margin";
    return finish(result);
  }
  const auto goal_idx = grid.nearestFree(goal_raw, params_.snap_radius);
  if (!goal_idx) {
    result.message = "goal is inside an obstacle, a no-fly zone, or its clearance margin";
    return finish(result);
  }

  const double k = params_.climb_cost;
  const double res = grid.resolution();
  const auto steps = makeSteps(res, k);
  const std::size_t goal_flat = grid.flatten(*goal_idx);
  const GridIndex g_idx = *goal_idx;
  auto heuristic = [&](const GridIndex & i) {
      const double dx = i.x - g_idx.x;
      const double dy = i.y - g_idx.y;
      const double dz = i.z - g_idx.z;
      return res * std::sqrt(dx * dx + dy * dy + k * k * dz * dz);
    };

  // Dense per-cell arrays: a few MB at planning resolutions and much faster than hashing.
  constexpr std::uint32_t kNoParent = std::numeric_limits<std::uint32_t>::max();
  const std::size_t n = grid.cellCount();
  std::vector<float> g(n, std::numeric_limits<float>::infinity());
  std::vector<std::uint32_t> parent(n, kNoParent);
  std::vector<std::uint8_t> closed(n, 0);

  using Entry = std::pair<double, std::uint32_t>;  // (f, cell)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  const std::size_t start_flat = grid.flatten(*start_idx);
  g[start_flat] = 0.0f;
  // A hair of extra weight on h breaks ties toward the goal and roughly halves expansions in
  // open space, at a cost bounded by 0.1% of path length.
  constexpr double kTieBreak = 1.001;
  open.emplace(heuristic(*start_idx) * kTieBreak, static_cast<std::uint32_t>(start_flat));

  bool found = false;
  while (!open.empty()) {
    const auto [f, cell] = open.top();
    open.pop();
    if (closed[cell]) {
      continue;
    }
    closed[cell] = 1;
    if (++result.iterations > params_.max_expansions) {
      result.message = "search limit reached before finding a path";
      return finish(result);
    }
    if (cell == goal_flat) {
      found = true;
      break;
    }
    const GridIndex cur = grid.unflatten(cell);
    const double g_cur = g[cell];
    for (const auto & s : steps) {
      const GridIndex nb{cur.x + s.dx, cur.y + s.dy, cur.z + s.dz};
      if (!grid.isFree(nb)) {
        continue;
      }
      const std::size_t nf = grid.flatten(nb);
      if (closed[nf]) {
        continue;
      }
      const double tentative = g_cur + s.cost;
      if (tentative < g[nf]) {
        g[nf] = static_cast<float>(tentative);
        parent[nf] = cell;
        open.emplace(tentative + heuristic(nb) * kTieBreak, static_cast<std::uint32_t>(nf));
      }
    }
  }

  if (!found) {
    result.message = "no path exists at this resolution and clearance";
    return finish(result);
  }

  Path cells;
  for (std::uint32_t c = static_cast<std::uint32_t>(goal_flat); c != kNoParent; c = parent[c]) {
    cells.push_back(grid.toWorld(grid.unflatten(c)));
  }
  // The vehicle is at `start` whether or not that cell is free, so the path always begins
  // there. The goal is different: if it had to be moved, ending at the requested point would
  // lead the vehicle back into the margin we just steered around, so the path ends at the
  // free cell and the caller decides whether that is close enough.
  const bool goal_moved = !(*goal_idx == goal_raw);
  result.path.reserve(cells.size() + 2);
  result.path.push_back(start);
  result.path.insert(result.path.end(), cells.rbegin(), cells.rend());
  if (!goal_moved) {
    result.path.push_back(goal);
  }
  result.success = true;
  result.message = "path found";
  if (!(*start_idx == start_raw)) {
    result.message += " (start moved to nearest free cell)";
  }
  if (goal_moved) {
    result.message += " (goal moved to nearest free cell)";
  }
  return finish(result);
}

}  // namespace autolivery_core
