#include "autolivery_core/rrt_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autolivery_core
{

namespace
{

struct TreeNode
{
  Vec3 p;
  int parent{-1};
  double cost{0.0};
  std::vector<int> children;
};

// After a rewire lowers a node's cost, every descendant gets cheaper by the same amount.
// Skipping this (as many textbook implementations do) leaves stale costs in the tree and
// the planner then picks parents and goal connections on wrong numbers.
void propagateCost(std::vector<TreeNode> & tree, int root, double delta)
{
  std::vector<int> stack{root};
  while (!stack.empty()) {
    const int i = stack.back();
    stack.pop_back();
    for (int c : tree[i].children) {
      tree[c].cost += delta;
      stack.push_back(c);
    }
  }
}

}  // namespace

RrtStarPlanner::RrtStarPlanner(std::shared_ptr<const WorldMap> map, RrtStarParams params)
: map_(std::move(map)), params_(params)
{
  if (!map_) {
    throw std::invalid_argument("RrtStarPlanner needs a map");
  }
  if (!(params_.step_size > 0.0) || !(params_.rewire_radius > 0.0)) {
    throw std::invalid_argument("step_size and rewire_radius must be positive");
  }
  if (params_.goal_bias < 0.0 || params_.goal_bias > 1.0) {
    throw std::invalid_argument("goal_bias must be in [0, 1]");
  }
}

PlanResult RrtStarPlanner::plan(const Vec3 & start, const Vec3 & goal) const
{
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  PlanResult result;
  const WorldMap & map = *map_;
  const double c = params_.clearance;
  const double k = params_.climb_cost;

  if (!map.inBounds(start)) {
    result.message = "start is outside the geofence";
    return result;
  }
  if (map.isBlocked(goal, c)) {
    result.message = "goal is inside an obstacle, a no-fly zone, or its clearance margin";
    result.planning_time = elapsed();
    return result;
  }
  // The start may legitimately sit inside a clearance margin (the vehicle is where it is);
  // growing the tree from it is still safe because every new edge is checked.
  if (map.segmentClear(start, goal, c)) {
    result.success = true;
    result.path = {start, goal};
    result.message = "direct path is clear";
    result.planning_time = elapsed();
    return result;
  }

  const Vec3 lo{
    map.boundsMin().x, map.boundsMin().y,
    std::max(map.boundsMin().z, params_.min_altitude)};
  const Vec3 hi{
    map.boundsMax().x, map.boundsMax().y,
    std::min(map.boundsMax().z, params_.max_altitude)};
  if (lo.z > hi.z) {
    result.message = "sampling altitude band does not intersect the geofence";
    return result;
  }

  std::mt19937 rng(params_.seed != 0 ? params_.seed : std::random_device{}());
  std::uniform_real_distribution<double> ux(lo.x, hi.x);
  std::uniform_real_distribution<double> uy(lo.y, hi.y);
  std::uniform_real_distribution<double> uz(lo.z, hi.z);
  std::uniform_real_distribution<double> coin(0.0, 1.0);

  std::vector<TreeNode> tree;
  tree.reserve(params_.max_iterations + 1);
  tree.push_back({start, -1, 0.0, {}});
  std::vector<int> goal_parents;  // nodes with a clear edge to the goal
  std::vector<int> near;

  for (; result.iterations < params_.max_iterations; ++result.iterations) {
    if ((result.iterations & 63U) == 0 && elapsed() > params_.max_planning_time) {
      break;
    }
    const Vec3 sample = coin(rng) < params_.goal_bias ? goal : Vec3{ux(rng), uy(rng), uz(rng)};

    int nearest = 0;
    double nearest_d = distance(tree[0].p, sample);
    for (int i = 1; i < static_cast<int>(tree.size()); ++i) {
      const double d = distance(tree[i].p, sample);
      if (d < nearest_d) {
        nearest_d = d;
        nearest = i;
      }
    }
    if (nearest_d < 1e-6) {
      continue;
    }
    const Vec3 new_p = nearest_d <= params_.step_size ?
      sample : tree[nearest].p + (sample - tree[nearest].p) * (params_.step_size / nearest_d);
    if (map.isBlocked(new_p, c) || !map.segmentClear(tree[nearest].p, new_p, c)) {
      continue;
    }

    near.clear();
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
      if (distance(tree[i].p, new_p) <= params_.rewire_radius) {
        near.push_back(i);
      }
    }

    int best_parent = nearest;
    double best_cost = tree[nearest].cost + weightedDistance(tree[nearest].p, new_p, k);
    for (int i : near) {
      const double cand = tree[i].cost + weightedDistance(tree[i].p, new_p, k);
      if (cand < best_cost && map.segmentClear(tree[i].p, new_p, c)) {
        best_cost = cand;
        best_parent = i;
      }
    }

    const int new_i = static_cast<int>(tree.size());
    tree.push_back({new_p, best_parent, best_cost, {}});
    tree[best_parent].children.push_back(new_i);

    for (int j : near) {
      if (j == best_parent) {
        continue;
      }
      const double via_new = best_cost + weightedDistance(new_p, tree[j].p, k);
      if (via_new + 1e-9 < tree[j].cost && map.segmentClear(new_p, tree[j].p, c)) {
        auto & siblings = tree[tree[j].parent].children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), j), siblings.end());
        const double delta = via_new - tree[j].cost;
        tree[j].parent = new_i;
        tree[j].cost = via_new;
        tree[new_i].children.push_back(j);
        propagateCost(tree, j, delta);
      }
    }

    if (distance(new_p, goal) <= params_.step_size && map.segmentClear(new_p, goal, c)) {
      goal_parents.push_back(new_i);
    }
  }

  if (goal_parents.empty()) {
    result.message = "no path found within " + std::to_string(result.iterations) +
      " samples; try A*, a larger iteration budget, or a smaller clearance";
    result.planning_time = elapsed();
    return result;
  }

  int best = goal_parents.front();
  double best_total = tree[best].cost + weightedDistance(tree[best].p, goal, k);
  for (int i : goal_parents) {
    const double total = tree[i].cost + weightedDistance(tree[i].p, goal, k);
    if (total < best_total) {
      best_total = total;
      best = i;
    }
  }

  Path reversed{goal};
  for (int i = best; i != -1; i = tree[i].parent) {
    reversed.push_back(tree[i].p);
  }
  result.path.assign(reversed.rbegin(), reversed.rend());
  result.success = true;
  result.message = "path found with " + std::to_string(tree.size()) + " tree nodes";
  result.planning_time = elapsed();
  return result;
}

}  // namespace autolivery_core
