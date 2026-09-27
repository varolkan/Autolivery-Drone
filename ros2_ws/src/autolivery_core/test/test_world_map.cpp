#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "autolivery_core/voxel_grid.hpp"
#include "autolivery_core/world_map.hpp"
#include "test_support.hpp"

using autolivery_core::Box;
using autolivery_core::Cylinder;
using autolivery_core::Vec3;
using autolivery_core::VoxelGrid;
using autolivery_core::WorldMap;

namespace
{

WorldMap singleBox()
{
  WorldMap map({-50.0, -50.0, 0.0}, {50.0, 50.0, 50.0});
  map.addBuilding({"block", {-5.0, -5.0, 0.0}, {5.0, 5.0, 20.0}});
  return map;
}

}  // namespace

TEST(WorldMap, GeofenceIsHardAndNotGrownByClearance)
{
  const WorldMap map = singleBox();
  EXPECT_TRUE(map.inBounds({0.0, 0.0, 30.0}));
  EXPECT_FALSE(map.inBounds({0.0, 0.0, 50.1}));
  EXPECT_FALSE(map.inBounds({-50.1, 0.0, 10.0}));
  // 1 m inside the fence is fine even with a 5 m clearance: clearance applies to obstacles.
  EXPECT_FALSE(map.isBlocked({49.0, 0.0, 10.0}, 5.0));
  EXPECT_TRUE(map.isBlocked({51.0, 0.0, 10.0}, 0.0));
}

TEST(WorldMap, ClearanceGrowsObstacles)
{
  const WorldMap map = singleBox();
  EXPECT_TRUE(map.isBlocked({0.0, 0.0, 10.0}, 0.0));
  EXPECT_FALSE(map.isBlocked({8.0, 0.0, 10.0}, 2.0));
  EXPECT_TRUE(map.isBlocked({8.0, 0.0, 10.0}, 3.5));
  EXPECT_FALSE(map.isBlocked({0.0, 0.0, 24.0}, 3.0));
  EXPECT_TRUE(map.isBlocked({0.0, 0.0, 22.0}, 3.0));
}

TEST(WorldMap, SegmentThroughBoxIsBlockedEvenWhenBothEndsAreClear)
{
  const WorldMap map = singleBox();
  // Both endpoints are far from the box; only an exact test catches the middle.
  EXPECT_FALSE(map.segmentClear({-30.0, 0.0, 10.0}, {30.0, 0.0, 10.0}, 0.0));
  EXPECT_EQ(map.firstBlocker({-30.0, 0.0, 10.0}, {30.0, 0.0, 10.0}, 0.0), "block");
  // Passing 6 m from the side is clear at 0.5 m clearance and blocked at 1.5 m.
  EXPECT_TRUE(map.segmentClear({-30.0, 11.0, 10.0}, {30.0, 11.0, 10.0}, 0.5));
  EXPECT_FALSE(map.segmentClear({-30.0, 6.0, 10.0}, {30.0, 6.0, 10.0}, 1.5));
  // Over the roof.
  EXPECT_TRUE(map.segmentClear({-30.0, 0.0, 25.0}, {30.0, 0.0, 25.0}, 2.0));
  // Diagonal that clips a corner.
  EXPECT_FALSE(map.segmentClear({-10.0, 0.0, 10.0}, {0.0, 10.0, 10.0}, 0.0));
  // Leaving the geofence.
  EXPECT_EQ(map.firstBlocker({0.0, 30.0, 10.0}, {0.0, 60.0, 10.0}, 0.0), "geofence");
}

TEST(WorldMap, CylinderSegments)
{
  WorldMap map({-50.0, -50.0, 0.0}, {50.0, 50.0, 50.0});
  map.addCylinder({"mast", {0.0, 0.0, 0.0}, 1.0, 30.0});
  EXPECT_FALSE(map.segmentClear({-10.0, 0.0, 10.0}, {10.0, 0.0, 10.0}, 0.0));
  EXPECT_TRUE(map.segmentClear({-10.0, 2.0, 10.0}, {10.0, 2.0, 10.0}, 0.5));
  EXPECT_FALSE(map.segmentClear({-10.0, 2.0, 10.0}, {10.0, 2.0, 10.0}, 1.5));
  EXPECT_TRUE(map.segmentClear({-10.0, 0.0, 35.0}, {10.0, 0.0, 35.0}, 2.0));
  // Descending diagonal that only enters the cylinder's height band near the mast.
  EXPECT_FALSE(map.segmentClear({-10.0, 0.0, 40.0}, {10.0, 0.0, 20.0}, 0.0));
  // Vertical segment beside it.
  EXPECT_TRUE(map.segmentClear({3.0, 0.0, 40.0}, {3.0, 0.0, 1.0}, 1.0));
  EXPECT_FALSE(map.segmentClear({1.5, 0.0, 40.0}, {1.5, 0.0, 1.0}, 1.0));
}

TEST(WorldMap, NoFlyZonesBlockLikeBuildings)
{
  WorldMap map({-50.0, -50.0, 0.0}, {50.0, 50.0, 50.0});
  map.addNoFlyZone({"school", {10.0, -10.0, 0.0}, {20.0, 10.0, 50.0}});
  EXPECT_TRUE(map.isBlocked({15.0, 0.0, 45.0}, 0.0));
  EXPECT_EQ(map.firstBlocker({0.0, 0.0, 30.0}, {40.0, 0.0, 30.0}, 1.0), "school");
  // No-fly zones are airspace, not ground: they do not raise the surface height.
  EXPECT_DOUBLE_EQ(map.groundHeight(15.0, 0.0), 0.0);
}

TEST(WorldMap, DistanceAndGroundHeight)
{
  const WorldMap map = singleBox();
  EXPECT_DOUBLE_EQ(map.distanceToObstacles({0.0, 0.0, 10.0}), 0.0);
  EXPECT_NEAR(map.distanceToObstacles({8.0, 0.0, 10.0}), 3.0, 1e-9);
  EXPECT_NEAR(map.distanceToObstacles({8.0, 9.0, 24.0}), std::sqrt(9.0 + 16.0 + 16.0), 1e-9);
  EXPECT_DOUBLE_EQ(map.groundHeight(0.0, 0.0), 20.0);
  EXPECT_DOUBLE_EQ(map.groundHeight(20.0, 0.0), 0.0);
  EXPECT_TRUE(std::isinf(WorldMap({0, 0, 0}, {1, 1, 1}).distanceToObstacles({0.5, 0.5, 0.5})));
}

TEST(WorldMap, ValidateCatchesBrokenGeometry)
{
  WorldMap map({-50.0, -50.0, 0.0}, {50.0, 50.0, 50.0});
  EXPECT_TRUE(map.validate().empty());
  map.addBuilding({"inside_out", {5.0, 5.0, 0.0}, {-5.0, 10.0, 10.0}});
  map.addCylinder({"flat", {0.0, 0.0, 0.0}, 0.0, 10.0});
  EXPECT_EQ(map.validate().size(), 2u);
  WorldMap empty({0.0, 0.0, 0.0}, {0.0, 10.0, 10.0});
  EXPECT_FALSE(empty.validate().empty());
}

TEST(VoxelGrid, InflatesObstaclesByClearancePlusHalfDiagonal)
{
  const WorldMap map = singleBox();
  const VoxelGrid grid(map, 1.0, 2.0);
  EXPECT_FALSE(grid.isFree(grid.toIndex({0.0, 0.0, 10.0})));
  EXPECT_FALSE(grid.isFree(grid.toIndex({7.0, 0.0, 10.0})));   // within clearance
  EXPECT_TRUE(grid.isFree(grid.toIndex({9.5, 0.0, 10.0})));    // beyond clearance + 0.87
  EXPECT_TRUE(grid.isFree(grid.toIndex({0.0, 0.0, 24.5})));    // above the roof margin

  // Every free voxel centre keeps the promised clearance.
  for (std::size_t f = 0; f < grid.cellCount(); ++f) {
    const auto idx = grid.unflatten(f);
    if (grid.isFree(idx)) {
      ASSERT_FALSE(map.isBlocked(grid.toWorld(idx), 2.0)) << "free voxel inside the margin";
    }
  }
}

TEST(VoxelGrid, IndexRoundTripAndNearestFree)
{
  const WorldMap map = singleBox();
  const VoxelGrid grid(map, 2.0, 1.0);
  const auto idx = grid.toIndex({13.0, -7.0, 21.0});
  EXPECT_EQ(grid.unflatten(grid.flatten(idx)), idx);
  const Vec3 c = grid.toWorld(idx);
  EXPECT_NEAR(c.x, 13.0, 1.0);
  EXPECT_NEAR(c.z, 21.0, 1.0);

  const auto inside = grid.toIndex({0.0, 0.0, 10.0});
  EXPECT_FALSE(grid.nearestFree(inside, 2).has_value());
  const auto near = grid.nearestFree(grid.toIndex({6.5, 0.0, 10.0}), 3);
  ASSERT_TRUE(near.has_value());
  EXPECT_TRUE(grid.isFree(*near));
}

TEST(VoxelGrid, RefusesAbsurdSizes)
{
  WorldMap huge({-100000.0, -100000.0, 0.0}, {100000.0, 100000.0, 500.0});
  EXPECT_THROW(VoxelGrid(huge, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelGrid(singleBox(), 0.0, 1.0), std::invalid_argument);
}

TEST(VoxelGrid, CityRasterisesQuickly)
{
  auto city = autolivery_test::makeCity();
  const VoxelGrid grid(*city, 2.0, 4.0);
  EXPECT_GT(grid.blockedCount(), 0u);
  EXPECT_LT(grid.blockedCount(), grid.cellCount() / 4);
}
