#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "autolivery_core/astar.hpp"
#include "autolivery_core/path_smoother.hpp"
#include "autolivery_core/rrt_star.hpp"
#include "autolivery_core/voxel_grid.hpp"
#include "test_support.hpp"

using namespace autolivery_core;  // NOLINT

namespace
{

constexpr double kClearance = 4.0;

void expectClear(const Path & path, const WorldMap & map, double clearance)
{
  ASSERT_GE(path.size(), 2u);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(map.segmentClear(path[i - 1], path[i], clearance))
      << "segment " << i << " hits " << map.firstBlocker(path[i - 1], path[i], clearance);
  }
}

double maxAltitude(const Path & path)
{
  double z = -1e9;
  for (const auto & p : path) {
    z = std::max(z, p.z);
  }
  return z;
}

class PlannerTest : public ::testing::Test
{
protected:
  std::shared_ptr<WorldMap> city = autolivery_test::makeCity();
  std::shared_ptr<VoxelGrid> grid = std::make_shared<VoxelGrid>(*city, 2.0, kClearance);
  Vec3 depot{0.0, 0.0, 40.0};
  Vec3 park{150.0, 110.0, 40.0};
};

}  // namespace

TEST_F(PlannerTest, AStarStraightLineInOpenSpace)
{
  AStarPlanner astar(grid);
  const Vec3 a{-200.0, -200.0, 40.0};
  const Vec3 b{-100.0, -150.0, 40.0};
  const auto r = astar.plan(a, b);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.path.front().x, a.x);
  EXPECT_EQ(r.path.back().x, b.x);
  // Grid paths are a little longer than the straight line, but not much.
  EXPECT_LT(pathLength(r.path), distance(a, b) * 1.1 + 4.0);
  expectClear(r.path, *city, kClearance);
}

TEST_F(PlannerTest, AStarGoesAroundTheTowerNotOverIt)
{
  AStarPlanner astar(grid);
  const auto r = astar.plan(depot, park);
  ASSERT_TRUE(r.success) << r.message;
  expectClear(r.path, *city, kClearance);
  // The tower is 80 m tall; climbing over it is far more expensive than the detour.
  EXPECT_LT(maxAltitude(r.path), 50.0);
  EXPECT_GT(pathLength(r.path), distance(depot, park));
}

TEST_F(PlannerTest, AStarClimbsWhenThereIsNoWayAround)
{
  auto map = std::make_shared<WorldMap>(Vec3{-100.0, -100.0, 0.0}, Vec3{100.0, 100.0, 80.0});
  map->addBuilding({"wall", {-5.0, -100.0, 0.0}, {5.0, 100.0, 30.0}});
  auto g = std::make_shared<VoxelGrid>(*map, 2.0, 3.0);
  AStarPlanner astar(g);
  const auto r = astar.plan({-50.0, 0.0, 20.0}, {50.0, 0.0, 20.0});
  ASSERT_TRUE(r.success) << r.message;
  expectClear(r.path, *map, 3.0);
  EXPECT_GT(maxAltitude(r.path), 33.0);
}

TEST_F(PlannerTest, AStarRefusesGoalsItCannotReach)
{
  AStarPlanner astar(grid);
  auto r = astar.plan(depot, {145.0, -115.0, 40.0});
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("no-fly"), std::string::npos) << r.message;

  r = astar.plan(depot, {1000.0, 0.0, 40.0});
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("geofence"), std::string::npos) << r.message;

  // A courtyard: the goal is free, but walled in on all sides up to the ceiling.
  auto map = std::make_shared<WorldMap>(Vec3{-60.0, -60.0, 0.0}, Vec3{60.0, 60.0, 40.0});
  map->addBuilding({"n", {-30.0, 20.0, 0.0}, {30.0, 30.0, 40.0}});
  map->addBuilding({"s", {-30.0, -30.0, 0.0}, {30.0, -20.0, 40.0}});
  map->addBuilding({"e", {20.0, -30.0, 0.0}, {30.0, 30.0, 40.0}});
  map->addBuilding({"w", {-30.0, -30.0, 0.0}, {-20.0, 30.0, 40.0}});
  AStarPlanner boxed(std::make_shared<VoxelGrid>(*map, 2.0, 2.0));
  r = boxed.plan({-50.0, -50.0, 20.0}, {0.0, 0.0, 20.0});
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("no path"), std::string::npos) << r.message;
}

TEST_F(PlannerTest, AStarMovesAStartInsideTheMarginOut)
{
  AStarPlanner astar(grid);
  // 3 m from the tower wall: inside the 4 m planning margin, but the vehicle is there.
  const Vec3 start{37.0, 50.0, 40.0};
  const auto r = astar.plan(start, {-50.0, 50.0, 40.0});
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.path.front().x, start.x);
  EXPECT_NE(r.message.find("start moved"), std::string::npos);
}

TEST_F(PlannerTest, AStarRejectsBadConfiguration)
{
  EXPECT_THROW(AStarPlanner(nullptr), std::invalid_argument);
  AStarParams p;
  p.climb_cost = 0.5;
  EXPECT_THROW(AStarPlanner(grid, p), std::invalid_argument);
}

TEST_F(PlannerTest, RrtStarFindsAClearPathAroundTheTower)
{
  RrtStarParams p;
  p.seed = 42;
  p.clearance = kClearance;
  p.min_altitude = 20.0;
  p.max_altitude = 70.0;
  p.max_planning_time = 5.0;
  RrtStarPlanner rrt(city, p);
  const auto r = rrt.plan(depot, park);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.path.front().x, depot.x);
  EXPECT_EQ(r.path.back().x, park.x);
  expectClear(r.path, *city, kClearance);

  // After smoothing, RRT* should be in the same league as the grid-optimal route.
  AStarPlanner astar(grid);
  const auto a = astar.plan(depot, park);
  ASSERT_TRUE(a.success);
  const double rrt_len = pathLength(shortcutPath(r.path, *city, kClearance));
  const double astar_len = pathLength(shortcutPath(a.path, *city, kClearance));
  EXPECT_LT(rrt_len, astar_len * 1.35);
}

TEST_F(PlannerTest, RrtStarIsDeterministicForAGivenSeed)
{
  RrtStarParams p;
  p.seed = 7;
  p.clearance = kClearance;
  p.min_altitude = 20.0;
  p.max_altitude = 70.0;
  p.max_iterations = 1500;
  p.max_planning_time = 30.0;  // iteration-bound, so timing cannot change the result
  RrtStarPlanner rrt(city, p);
  const auto a = rrt.plan(depot, park);
  const auto b = rrt.plan(depot, park);
  ASSERT_TRUE(a.success);
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_EQ(a.path[i].x, b.path[i].x);
    EXPECT_EQ(a.path[i].y, b.path[i].y);
  }
}

TEST_F(PlannerTest, RrtStarShortCircuitsAClearLine)
{
  RrtStarPlanner rrt(city);
  const auto r = rrt.plan({-200.0, -200.0, 40.0}, {-100.0, -150.0, 40.0});
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.size(), 2u);
}

TEST_F(PlannerTest, RrtStarRefusesBlockedGoalAndBadParams)
{
  RrtStarParams p;
  p.clearance = kClearance;
  RrtStarPlanner rrt(city, p);
  EXPECT_FALSE(rrt.plan(depot, {55.0, 50.0, 40.0}).success);  // inside the tower
  p.step_size = 0.0;
  EXPECT_THROW(RrtStarPlanner(city, p), std::invalid_argument);
  EXPECT_THROW(RrtStarPlanner(nullptr), std::invalid_argument);
}

TEST_F(PlannerTest, RrtStarRespectsTheTimeBudget)
{
  RrtStarParams p;
  p.clearance = kClearance;
  p.max_iterations = 10'000'000;
  p.max_planning_time = 0.3;
  p.min_altitude = 20.0;
  p.max_altitude = 70.0;
  RrtStarPlanner rrt(city, p);
  const auto r = rrt.plan(depot, park);
  EXPECT_LT(r.planning_time, 1.0);
}
