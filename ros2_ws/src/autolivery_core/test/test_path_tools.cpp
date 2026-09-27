#include <gtest/gtest.h>

#include <memory>

#include "autolivery_core/astar.hpp"
#include "autolivery_core/path_follower.hpp"
#include "autolivery_core/path_smoother.hpp"
#include "autolivery_core/voxel_grid.hpp"
#include "test_support.hpp"

using namespace autolivery_core;  // NOLINT

TEST(PathSmoother, ShortcutCollapsesGridStaircaseAndStaysClear)
{
  auto city = autolivery_test::makeCity();
  auto grid = std::make_shared<VoxelGrid>(*city, 2.0, 4.0);
  AStarPlanner astar(grid);
  const Vec3 a{0.0, 0.0, 40.0};
  const Vec3 b{150.0, 110.0, 40.0};
  const auto raw = astar.plan(a, b);
  ASSERT_TRUE(raw.success);

  const Path smooth = shortcutPath(raw.path, *city, 4.0);
  EXPECT_LT(smooth.size(), raw.path.size());
  EXPECT_LE(smooth.size(), 6u);
  EXPECT_LE(pathLength(smooth), pathLength(raw.path) + 1e-9);
  EXPECT_EQ(smooth.front().x, a.x);
  EXPECT_EQ(smooth.back().x, b.x);
  for (std::size_t i = 1; i < smooth.size(); ++i) {
    EXPECT_TRUE(city->segmentClear(smooth[i - 1], smooth[i], 4.0));
  }
}

TEST(PathSmoother, ShortcutKeepsSegmentsItCannotImprove)
{
  WorldMap map({-50.0, -50.0, 0.0}, {50.0, 50.0, 50.0});
  map.addBuilding({"b", {-2.0, -2.0, 0.0}, {2.0, 2.0, 40.0}});
  const Path around{{-10.0, 0.0, 10.0}, {0.0, 10.0, 10.0}, {10.0, 0.0, 10.0}};
  EXPECT_EQ(shortcutPath(around, map, 1.0).size(), 3u);
  const Path trivial{{0.0, 0.0, 45.0}, {1.0, 0.0, 45.0}};
  EXPECT_EQ(shortcutPath(trivial, map, 1.0).size(), 2u);
}

TEST(PathSmoother, DensifyBoundsSegmentLength)
{
  const Path p{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 3.0, 0.0}};
  const Path d = densifyPath(p, 2.5);
  for (std::size_t i = 1; i < d.size(); ++i) {
    EXPECT_LE(distance(d[i - 1], d[i]), 2.5 + 1e-9);
  }
  EXPECT_NEAR(pathLength(d), pathLength(p), 1e-9);
  EXPECT_EQ(d.back().y, 3.0);
  EXPECT_EQ(densifyPath(p, 0.0).size(), p.size());
}

TEST(PathSmoother, RemoveDuplicatesKeepsEndpoints)
{
  const Path p{{0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.001, 0.0, 0.0}};
  const Path r = removeDuplicates(p, 0.1);
  ASSERT_EQ(r.size(), 2u);
  EXPECT_EQ(r.front().x, 0.0);
  EXPECT_EQ(r.back().x, 5.001);
}

TEST(PathFollower, CarrotLeadsAlongThePathAndRoundsCorners)
{
  PathFollower f;
  f.reset({{0.0, 0.0, 10.0}, {20.0, 0.0, 10.0}, {20.0, 20.0, 10.0}});
  EXPECT_DOUBLE_EQ(f.totalLength(), 40.0);

  auto p = f.update({5.0, 1.0, 10.0}, 8.0, 1.0);
  EXPECT_EQ(p.segment, 0u);
  EXPECT_NEAR(p.carrot.x, 13.0, 1e-9);
  EXPECT_NEAR(p.cross_track, 1.0, 1e-9);
  EXPECT_NEAR(p.remaining, 35.0, 1e-9);
  EXPECT_FALSE(p.finished);

  // Near the corner the carrot is already around it.
  p = f.update({17.0, 0.0, 10.0}, 8.0, 1.0);
  EXPECT_NEAR(p.carrot.x, 20.0, 1e-9);
  EXPECT_NEAR(p.carrot.y, 5.0, 1e-9);

  // Within acceptance of the corner: switch segments.
  p = f.update({19.5, 0.2, 10.0}, 8.0, 1.0);
  EXPECT_EQ(p.segment, 1u);
  EXPECT_TRUE(p.final_segment);

  p = f.update({20.0, 19.5, 10.0}, 8.0, 1.0);
  EXPECT_TRUE(p.finished);
}

TEST(PathFollower, JoinsTheNextLegWhenItCutsTheCorner)
{
  // A sharp corner flown wide: the vehicle is on the second leg, well away from the corner
  // and short of the end of the first leg. It must follow the second leg, not the first.
  PathFollower f;
  f.reset({{0.0, 0.0, 30.0}, {300.0, 250.0, 30.0}, {-50.0, 250.0, 30.0}});
  auto p = f.update({230.0, 248.0, 30.0}, 100.0, 20.0);
  EXPECT_EQ(p.segment, 1u);
  EXPECT_TRUE(p.final_segment);
  EXPECT_LT(p.carrot.x, 230.0);  // carrot ahead along the second leg, westward

  // But not before the vehicle has reached the next leg's span.
  PathFollower g;
  g.reset({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 5.0, 0.0}});
  EXPECT_EQ(g.update({0.0, 0.0, 0.0}, 2.0, 1.0).segment, 0u);
}

TEST(PathFollower, NeverGoesBackwardsAndHandlesDegeneratePaths)
{
  PathFollower f;
  f.reset({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}});
  f.update({10.0, 5.0, 0.0}, 2.0, 1.0);  // jumped onto the second leg
  const auto p = f.update({2.0, 0.0, 0.0}, 2.0, 1.0);
  EXPECT_EQ(p.segment, 1u);

  PathFollower single;
  single.reset({{1.0, 2.0, 3.0}});
  EXPECT_TRUE(single.update({1.0, 2.0, 3.5}, 5.0, 1.0).finished);
  PathFollower none;
  EXPECT_TRUE(none.update({}, 5.0, 1.0).finished);
}
