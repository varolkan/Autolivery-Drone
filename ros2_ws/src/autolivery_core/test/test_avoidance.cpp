#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "autolivery_core/velocity_obstacle.hpp"

using namespace autolivery_core;  // NOLINT

namespace
{

AvoidanceParams params()
{
  AvoidanceParams p;
  p.vehicle_radius = 0.6;
  p.safety_margin = 3.0;
  p.max_horizontal_speed = 10.0;
  return p;
}

}  // namespace

TEST(Avoidance, NothingAroundMeansNoChange)
{
  VelocityObstacleAvoider avoider(params());
  const auto r = avoider.compute({0.0, 0.0, 30.0}, {8.0, 0.0, 0.0}, {}, nullptr);
  EXPECT_FALSE(r.modified);
  EXPECT_TRUE(r.conflict_free);
  EXPECT_DOUBLE_EQ(r.velocity.x, 8.0);
}

TEST(Avoidance, TrafficMovingAwayIsIgnored)
{
  VelocityObstacleAvoider avoider(params());
  const std::vector<DynamicObstacle> traffic{{"away", {30.0, 0.0, 30.0}, {12.0, 0.0, 0.0}, 1.0}};
  EXPECT_FALSE(avoider.compute({0.0, 0.0, 30.0}, {8.0, 0.0, 0.0}, traffic, nullptr).modified);
  const std::vector<DynamicObstacle> behind{{"behind", {-30.0, 0.0, 30.0}, {-5.0, 0.0, 0.0}, 1.0}};
  EXPECT_FALSE(avoider.compute({0.0, 0.0, 30.0}, {8.0, 0.0, 0.0}, behind, nullptr).modified);
}

TEST(Avoidance, HeadOnTrafficForcesASafeDeviation)
{
  VelocityObstacleAvoider avoider(params());
  const std::vector<DynamicObstacle> traffic{{"intruder", {60.0, 0.0, 30.0}, {-8.0, 0.0, 0.0},
    1.0}};
  const auto r = avoider.compute({0.0, 0.0, 30.0}, {8.0, 0.0, 0.0}, traffic, nullptr);
  EXPECT_TRUE(r.modified);
  EXPECT_TRUE(r.conflict_free);
  EXPECT_EQ(r.threat, "intruder");
  EXPECT_GE(r.min_separation, 0.0);
  // Still broadly heading for the goal rather than turning back.
  EXPECT_GT(r.velocity.x, 0.0);
}

TEST(Avoidance, SimulatedEncountersKeepSeparation)
{
  // Fly both aircraft forward in time with the avoider in the loop, from several geometries,
  // and check the closest approach actually achieved.
  const AvoidanceParams p = params();
  VelocityObstacleAvoider avoider(p);
  const double required = 1.0 + p.vehicle_radius + p.safety_margin;
  struct Encounter { Vec3 start; Vec3 velocity; };
  const std::vector<Encounter> encounters{
    {{80.0, 0.0, 30.0}, {-8.0, 0.0, 0.0}},     // head-on
    {{40.0, -40.0, 30.0}, {0.0, 8.0, 0.0}},    // crossing from the right
    {{40.0, 40.0, 30.0}, {0.0, -8.0, 0.0}},    // crossing from the left
    {{60.0, 0.0, 45.0}, {-6.0, 0.0, -1.5}},    // descending towards us
    {{30.0, 0.0, 30.0}, {0.0, 0.0, 0.0}},      // hovering on the route
  };
  for (const auto & e : encounters) {
    Vec3 me{0.0, 0.0, 30.0};
    Vec3 other = e.start;
    const Vec3 goal{160.0, 0.0, 30.0};
    double closest = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 600; ++i) {
      const double dt = 0.05;
      const Vec3 desired = (goal - me).normalized() * 8.0;
      const auto r = avoider.compute(me, desired, {{"x", other, e.velocity, 1.0}}, nullptr);
      me += r.velocity * dt;
      other += e.velocity * dt;
      closest = std::min(closest, distance(me, other));
    }
    // Allow a small discretisation loss from the 20 Hz loop.
    EXPECT_GT(closest, required - 0.5)
      << "encounter from (" << e.start.x << ", " << e.start.y << ", " << e.start.z << ")";
  }
}

TEST(Avoidance, ManoeuvresStayOutOfTheStaticMap)
{
  // A wall immediately to the left and a head-on intruder: the avoider must not dodge left.
  WorldMap map({-100.0, -100.0, 0.0}, {200.0, 100.0, 60.0});
  map.addBuilding({"wall", {0.0, 3.0, 0.0}, {200.0, 10.0, 60.0}});
  VelocityObstacleAvoider avoider(params());
  const std::vector<DynamicObstacle> traffic{{"intruder", {50.0, 0.0, 30.0}, {-8.0, 0.0, 0.0},
    1.0}};
  const auto r = avoider.compute({0.0, 0.0, 30.0}, {8.0, 0.0, 0.0}, traffic, &map);
  EXPECT_TRUE(r.modified);
  EXPECT_LE(r.velocity.y, 1e-9);
  EXPECT_TRUE(map.segmentClear({0.0, 0.0, 30.0}, Vec3{0.0, 0.0, 30.0} + r.velocity * 3.0, 0.5));
}

TEST(Avoidance, FixedWingNeverSlowsBelowStall)
{
  AvoidanceParams p = params();
  p.min_speed = 11.0;
  p.max_horizontal_speed = 18.0;
  VelocityObstacleAvoider avoider(p);
  const std::vector<DynamicObstacle> traffic{{"intruder", {80.0, 0.0, 30.0}, {-15.0, 0.0, 0.0},
    2.0}};
  const auto r = avoider.compute({0.0, 0.0, 30.0}, {15.0, 0.0, 0.0}, traffic, nullptr);
  EXPECT_TRUE(r.modified);
  EXPECT_GE(r.velocity.horizontalNorm(), 11.0 - 1e-9);
}

TEST(Avoidance, AlreadyTooCloseMovesAway)
{
  VelocityObstacleAvoider avoider(params());
  const std::vector<DynamicObstacle> traffic{{"close", {2.0, 0.0, 30.0}, {0.0, 0.0, 0.0}, 1.0}};
  const auto r = avoider.compute({0.0, 0.0, 30.0}, {5.0, 0.0, 0.0}, traffic, nullptr);
  EXPECT_TRUE(r.modified);
  EXPECT_FALSE(r.conflict_free);
  // Whatever it picked must increase the distance.
  const Vec3 after = Vec3{0.0, 0.0, 30.0} + r.velocity * 0.5;
  EXPECT_GT(distance(after, {2.0, 0.0, 30.0}), 2.0);
}
