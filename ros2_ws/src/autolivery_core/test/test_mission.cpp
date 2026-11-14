// End-to-end mission scenarios against the kinematic FakeVehicle: every airframe class and
// dropoff method, and every way a delivery can go wrong that the mission has to survive.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "autolivery_core/mission.hpp"
#include "autolivery_core/velocity_obstacle.hpp"
#include "test_support.hpp"

using namespace autolivery_core;  // NOLINT
using autolivery_test::config;
using autolivery_test::dump;
using autolivery_test::expectNoCollisions;
using autolivery_test::FakeVehicle;
using autolivery_test::makeMission;
using autolivery_test::RunRecord;
using autolivery_test::visited;
using autolivery_test::visitedInOrder;

namespace
{

// A planner that always hands back the same route, one way or the other: for tests that need
// a route of an exact shape rather than whatever the search finds.
PlanFn fixedRoute(const Path & route)
{
  return [route](const Vec3 & from, const Vec3 &) {
           PlanResult r;
           r.success = true;
           r.path = distance(from, route.front()) < distance(from, route.back()) ?
             route : Path(route.rbegin(), route.rend());
           return r;
         };
}

double offRoute(const Vec3 & p, const Path & route)
{
  double off = 1e9;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Vec3 on = lerp(route[i - 1], route[i], projectOntoSegment(p, route[i - 1], route[i]));
    off = std::min(off, distance(p, on));
  }
  return off;
}

}  // namespace

class MissionTest : public ::testing::Test
{
protected:
  std::shared_ptr<WorldMap> city = autolivery_test::makeCity();
};

TEST_F(MissionTest, QuadcopterHoverDeliveryToThePark)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);

  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 600.0);

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
  EXPECT_TRUE(mission.result().delivered);
  EXPECT_TRUE(
    visitedInOrder(
      r, {
    MissionState::Preflight, MissionState::Takeoff, MissionState::Outbound,
    MissionState::Descend, MissionState::Release, MissionState::Ascend, MissionState::Return,
    MissionState::Land, MissionState::Complete})) << dump(r);

  // The parcel came off over the dropoff, at release height.
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_LT(horizontalDistance(*vehicle.release_position, autolivery_test::kPark), 1.5);
  EXPECT_NEAR(
    vehicle.release_position->z, autolivery_test::kPark.z + profile.release_altitude,
    1.0);
  EXPECT_EQ(vehicle.release_commands, 1);

  // Home, on the pad (it stops over the pad before AUTO.LAND), motors off.
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 1.2);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  // PX4 put itself back in OFFBOARD after the touchdown disarm (found in SITL). That is not a
  // takeover, and the mission must have finished cleanly regardless.
  EXPECT_EQ(vehicle.mode, px4_mode::kOffboard);
  expectNoCollisions(vehicle, *city);
  EXPECT_GT(
    mission.result().distance_flown,
    2.0 * distance(autolivery_test::kHome, autolivery_test::kPark));
  EXPECT_GT(mission.result().flight_time, 30.0);
}

TEST_F(MissionTest, RooftopDropoffDescendsToTheRoofNotTheStreet)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kShopRoof, "shop roof"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 600.0);
  ASSERT_TRUE(mission.result().success) << dump(r);
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_NEAR(vehicle.release_position->z, 15.0 + profile.release_altitude, 1.0);
  expectNoCollisions(vehicle, *city);
}

TEST_F(MissionTest, CornersAreNotCutIntoThePlannedClearance)
{
  // A route that runs up the west face of a block and turns east along its north face, both
  // at exactly the planned clearance, as a planner would lay it round a building that is in
  // the way. A vehicle flying straight at a point further along the route takes the inside of
  // that corner, which is where the building is.
  const auto profile = autolivery_test::quadProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{400.0, 400.0, 120.0});
  map->addBuilding({"block", {60.0, -100.0, 0.0}, {100.0, 40.0, 80.0}});
  const double z = profile.cruise_altitude;
  const double off = profile.planning_clearance;
  const Path route{{60.0 - off, -130.0, z}, {60.0 - off, 40.0 + off, z}, {140.0, 40.0 + off, z}};
  MissionConfig cfg = config();
  cfg.home = {route.front().x, route.front().y, 0.0};
  cfg.lookahead = 16.0;  // a long view ahead, which is what cuts a corner deepest
  Mission mission(profile, cfg, map, fixedRoute(route));
  FakeVehicle vehicle(profile, cfg.home, map);
  ASSERT_EQ(mission.start({{140.0, 40.0 + off, 0.0}, "yard"}, vehicle.snapshot(), 0.0), "");

  double closest = 1e9;
  double widest = 0.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound || m.state() == MissionState::Return) {
        closest = std::min(closest, map->distanceToObstacles(v.position()));
        widest = std::max(widest, offRoute(v.position(), route));
      }
    });
  ASSERT_TRUE(mission.result().success) << dump(r);
  // Both ways round the corner it kept at least four fifths of the clearance it planned...
  EXPECT_GT(closest, 0.8 * off) << "came within " << closest << " m of the block";
  // ...and did not buy that by swinging wide on the outside of the turn, where in a street
  // the next building would be.
  EXPECT_LT(widest, 0.5 * off) << "swung " << widest << " m off the route";
  expectNoCollisions(vehicle, *map);
}

TEST_F(MissionTest, SlowsDownForACornerInProportionToTheTurn)
{
  // Open sky, nothing to hit: how a corner is taken is all there is to see.
  const auto profile = autolivery_test::quadProfile();  // cruise 8 m/s
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{400.0, 400.0, 120.0});
  const double z = profile.cruise_altitude;
  const auto fly = [&](const Path & route, double * slowest_at_corner, double * widest) {
      MissionConfig cfg = config();
      cfg.home = {route.front().x, route.front().y, 0.0};
      Mission mission(profile, cfg, map, fixedRoute(route));
      FakeVehicle vehicle(profile, cfg.home, map);
      EXPECT_EQ(
        mission.start(
          {{route.back().x, route.back().y, 0.0}, "field"}, vehicle.snapshot(), 0.0), "");
      *slowest_at_corner = 1e9;
      *widest = 0.0;
      const RunRecord r = autolivery_test::run(
        mission, vehicle, 600.0, [&](double, FakeVehicle & v, Mission & m) {
          if (m.state() != MissionState::Outbound) {
            return;
          }
          *widest = std::max(*widest, offRoute(v.position(), route));
          if (distance(v.position(), route[1]) < 10.0) {
            *slowest_at_corner = std::min(*slowest_at_corner, v.velocity().norm());
          }
        });
      EXPECT_TRUE(mission.result().success) << dump(r);
    };

  double slowest = 0.0;
  double widest = 0.0;
  // A right angle is taken at half the cruise speed or less. The vehicle still leaves the
  // route there, by the couple of metres its carrot rounds the inside of the corner; what it
  // must not do is arrive too fast to turn and swing wide of it (5 m, at cruise speed).
  fly({{0.0, 0.0, z}, {200.0, 0.0, z}, {200.0, 200.0, z}}, &slowest, &widest);
  EXPECT_LT(slowest, 0.6 * profile.cruise_speed);
  EXPECT_LT(widest, 3.0) << "left the route by " << widest << " m at the corner";
  // A ten-degree bend is not worth slowing down for.
  const Vec3 after_bend{200.0 + 200.0 * std::cos(0.1745), 200.0 * std::sin(0.1745), z};
  fly({{0.0, 0.0, z}, {200.0, 0.0, z}, after_bend}, &slowest, &widest);
  EXPECT_GT(slowest, 0.95 * profile.cruise_speed);
  EXPECT_LT(widest, 1.0);

  // A waypoint that appears twice in a route is not a corner of its own...
  fly({{0.0, 0.0, z}, {200.0, 0.0, z}, {200.0, 0.0, z}, {400.0, 0.0, z}}, &slowest, &widest);
  EXPECT_GT(slowest, 0.95 * profile.cruise_speed);
  // ...and does not hide the one that is there.
  fly({{0.0, 0.0, z}, {200.0, 0.0, z}, {200.0, 0.0, z}, {200.0, 200.0, z}}, &slowest, &widest);
  EXPECT_LT(slowest, 0.6 * profile.cruise_speed);
  EXPECT_LT(widest, 3.0);
}

TEST_F(MissionTest, VtolTakesCornersAsAMulticopter)
{
  // Two long straight stretches with a right-angle corner between them, round a block taller
  // than the cruise altitude and exactly the planned clearance away. A VTOL that stayed a
  // plane through the corner would turn on its radius, straight through the block.
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 700.0, 120.0});
  const double z = profile.cruise_altitude;
  const double off = profile.planning_clearance;
  map->addBuilding({"block", {400.0, off, 0.0}, {500.0 - off, 100.0, 90.0}});
  const Vec3 corner{500.0, 0.0, z};
  const Path route{{0.0, 0.0, z}, corner, {500.0, 450.0, z}};
  MissionConfig cfg = config();
  // Little room to transition in: it is still turning back into a multicopter with the
  // corner less than a plane's look-ahead away.
  cfg.vtol_transition_margin = 100.0;
  Mission mission(profile, cfg, map, fixedRoute(route));
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  ASSERT_EQ(mission.start({{500.0, 450.0, 0.0}, "farm"}, vehicle.snapshot(), 0.0), "");

  double closest = 1e9;
  bool plane_at_the_corner = false;
  int plane_stretches = 0;
  VtolState last = VtolState::Mc;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 1200.0, [&](double, FakeVehicle & v, Mission & m) {
      if (m.state() != MissionState::Outbound && m.state() != MissionState::Return) {
        return;
      }
      closest = std::min(closest, map->distanceToObstacles(v.position()));
      if (distance(v.position(), corner) < 40.0 && v.vtol() != VtolState::Mc) {
        plane_at_the_corner = true;
      }
      if (v.vtol() == VtolState::Fw && last != VtolState::Fw) {
        ++plane_stretches;
      }
      last = v.vtol();
    });

  ASSERT_TRUE(mission.result().success) << dump(r);
  EXPECT_FALSE(plane_at_the_corner);
  // A plane on each of the two stretches, there and back.
  EXPECT_EQ(plane_stretches, 4);
  EXPECT_GT(closest, 0.8 * off) << "came within " << closest << " m of the block";
  expectNoCollisions(vehicle, *map);
}

TEST_F(MissionTest, LandDropoffLandsReleasesAndTakesOffAgain)
{
  auto profile = autolivery_test::quadProfile();
  profile.dropoff_method = DropoffMethod::Land;
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 800.0);

  ASSERT_TRUE(mission.result().success) << dump(r);
  EXPECT_TRUE(
    visitedInOrder(
      r, {
    MissionState::Outbound, MissionState::Descend, MissionState::Release,
    MissionState::Relaunch, MissionState::Ascend, MissionState::Return})) << dump(r);
  ASSERT_TRUE(vehicle.release_position.has_value());
  // Released on the ground at the dropoff.
  EXPECT_NEAR(vehicle.release_position->z, 0.0, 0.1);
  EXPECT_LT(horizontalDistance(*vehicle.release_position, autolivery_test::kPark), 1.2);
  expectNoCollisions(vehicle, *city);
}

TEST_F(MissionTest, WinchDeliveryWithoutPayloadSensorTrustsTheTiming)
{
  auto profile = autolivery_test::quadProfile();
  profile.dropoff_method = DropoffMethod::Winch;
  profile.release_altitude = 12.0;
  MissionConfig cfg = config();
  cfg.payload_feedback = false;
  cfg.release_duration = 12.0 / 0.5 + 1.5;  // what releaseDuration() gives the ROS layer
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.payload_sensor = false;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 800.0);
  ASSERT_TRUE(mission.result().success) << dump(r);
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_NEAR(vehicle.release_position->z, 12.0, 1.0);
  EXPECT_EQ(vehicle.release_commands, 1);
}

TEST_F(MissionTest, FixedWingAirDropHitsTheTarget)
{
  const auto profile = autolivery_test::planeProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.runway_heading = kPi;  // the test city is open to the west
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 900.0);

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << dump(r);
  EXPECT_FALSE(visited(r, MissionState::Descend));

  // Where did the parcel land? Drag-free fall from the release state.
  ASSERT_TRUE(vehicle.release_position.has_value());
  const Vec3 p = *vehicle.release_position;
  const Vec3 v = *vehicle.release_velocity;
  const double h = p.z - autolivery_test::kPark.z;
  // The fake releases in the same tick as the command, so no actuation latency here.
  const double t = (v.z + std::sqrt(v.z * v.z + 2.0 * kGravity * h)) / kGravity;
  const Vec3 impact = p.horizontal() + v.horizontal() * t;
  EXPECT_LT(horizontalDistance(impact, autolivery_test::kPark), 10.0)
    << "impact at (" << impact.x << ", " << impact.y << ")";
  EXPECT_NEAR(h, profile.release_altitude, 5.0);
}

TEST_F(MissionTest, FixedWingLandsThroughTheMissionLandingPattern)
{
  const auto profile = autolivery_test::planeProfile();
  MissionConfig cfg = config();
  cfg.fw_mission_landing = true;
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.runway_heading = kPi;
  vehicle.fixed_wing_rtl_lands = true;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  bool saw_rtl = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double, FakeVehicle & v, Mission &) {
      saw_rtl = saw_rtl || v.mode == px4_mode::kRtl;
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
  EXPECT_TRUE(saw_rtl);
  EXPECT_TRUE(vehicle.onGround());
}

TEST_F(MissionTest, FixedWingWithoutARouteHomeUsesRtlThenLands)
{
  // Cancel mid-outbound with a planner that has stopped finding routes: the mission falls
  // back to the autopilot's RTL. PX4's fixed-wing RTL only loiters over home, so the mission
  // must notice and hand over to AUTO.LAND (a circular landing) itself.
  const auto profile = autolivery_test::planeProfile();
  auto inner = autolivery_test::makePlanner(city, profile.planning_clearance);
  bool planner_broken = false;
  auto planner = [&](const Vec3 & a, const Vec3 & b) {
      if (planner_broken) {
        PlanResult r;
        r.message = "planner offline";
        return r;
      }
      return inner(a, b);
    };
  Mission mission(profile, config(), city, planner);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.runway_heading = kPi;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  bool cancelled = false;
  bool saw_rtl = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double, FakeVehicle & v, Mission & m) {
      if (!cancelled && m.state() == MissionState::Outbound) {
        planner_broken = true;
        m.cancel();
        cancelled = true;
      }
      saw_rtl = saw_rtl || v.mode == px4_mode::kRtl;
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(saw_rtl);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  EXPECT_LT(
    horizontalDistance(vehicle.position(), autolivery_test::kHome),
    3.0 * profile.turn_radius);
}

TEST_F(MissionTest, VtolCruisesAsAPlaneAndDeliversAsAMulticopter)
{
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 400.0, 120.0});
  map->addBuilding({"block", {250.0, -40.0, 0.0}, {300.0, 40.0, 90.0}});
  Mission mission = makeMission(profile, map);
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  const Vec3 far_dropoff{700.0, 50.0, 0.0};
  ASSERT_EQ(mission.start({far_dropoff, "farm"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 900.0);

  ASSERT_TRUE(mission.result().success) << dump(r);
  EXPECT_TRUE(r.requested_fw);
  EXPECT_TRUE(vehicle.saw_fixed_wing);
  EXPECT_EQ(vehicle.vtol(), VtolState::Mc);
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_LT(horizontalDistance(*vehicle.release_position, far_dropoff), 1.5);
  expectNoCollisions(vehicle, *map);
}

// PX4's quad-chute: a front transition that does not build airspeed in time is given up,
// the vehicle is a multicopter again, and the fixed-wing side counts as failed. Under OFFBOARD
// PX4 only warns, so it is the mission that has to stop asking for plane flight.
TEST_F(MissionTest, VtolQuadchuteOnTheWayOutBringsTheParcelHome)
{
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 400.0, 120.0});
  Mission mission = makeMission(profile, map);
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  vehicle.quadchute = true;
  ASSERT_EQ(mission.start({{700.0, 50.0, 0.0}, "farm"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 900.0);

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_EQ(vehicle.quadchutes, 1) << "asked for plane flight again after the quad-chute";
  EXPECT_FALSE(vehicle.saw_fixed_wing);
  EXPECT_FALSE(mission.result().success);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(
    mission.result().message.find("in the transition to plane flight (quad-chute)"),
    std::string::npos) << mission.result().message;
  EXPECT_NE(mission.result().message.find("abandoning delivery"), std::string::npos);
  EXPECT_FALSE(visited(r, MissionState::Descend)) << dump(r);
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
  EXPECT_EQ(vehicle.release_commands, 0);
  EXPECT_EQ(vehicle.vtol(), VtolState::Mc);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 1.5);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  expectNoCollisions(vehicle, *map);
}

TEST_F(MissionTest, VtolQuadchuteOnTheWayBackFliesHomeAsAMulticopter)
{
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 400.0, 120.0});
  Mission mission = makeMission(profile, map);
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  ASSERT_EQ(mission.start({{700.0, 50.0, 0.0}, "farm"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Return) {
        v.quadchute = true;
      }
    });

  ASSERT_TRUE(mission.result().success) << dump(r);
  EXPECT_TRUE(mission.result().delivered);
  EXPECT_TRUE(vehicle.saw_fixed_wing);  // the way out was flown as a plane
  EXPECT_EQ(vehicle.quadchutes, 1);
  const std::string carried_on = "(quad-chute); flying the rest of the way as a multicopter";
  EXPECT_TRUE(
    std::any_of(
      r.events.begin(), r.events.end(),
      [&](const std::string & e) {return e.find(carried_on) != std::string::npos;})) << dump(r);
  EXPECT_EQ(vehicle.vtol(), VtolState::Mc);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 1.5);
  EXPECT_TRUE(vehicle.onGround());
  expectNoCollisions(vehicle, *map);
}

TEST_F(MissionTest, VtolDroppingOutOfPlaneFlightIsAQuadchuteToo)
{
  // Already a plane when PX4 gives up on it (too low, pitched or rolled too far).
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 400.0, 120.0});
  Mission mission = makeMission(profile, map);
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  ASSERT_EQ(mission.start({{700.0, 50.0, 0.0}, "farm"}, vehicle.snapshot(), 0.0), "");
  bool dropped_out = false;
  double plane_since = -1.0;
  int plane_stretches = 0;
  VtolState last = VtolState::Mc;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double t, FakeVehicle & v, Mission &) {
      if (v.vtol() == VtolState::Fw && last != VtolState::Fw) {
        ++plane_stretches;
        plane_since = t;
      }
      last = v.vtol();
      if (v.vtol() == VtolState::Fw && !dropped_out && t - plane_since > 5.0) {
        dropped_out = true;
        v.requestVtol(VtolRequest::Mc, t);  // the autopilot's doing, not the mission's
      }
    });

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(dropped_out);
  EXPECT_EQ(plane_stretches, 1) << "flew as a plane again after the quad-chute";
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(
    mission.result().message.find("from plane flight (quad-chute)"),
    std::string::npos) << mission.result().message;
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 1.5);
  EXPECT_TRUE(vehicle.onGround());
  expectNoCollisions(vehicle, *map);
}

TEST_F(MissionTest, VtolCruisesAsAPlaneAgainOnTheNextDelivery)
{
  // PX4 will not arm again after a quad-chute until it is rebooted; once it has been, the next
  // delivery starts afresh and may be a plane on the straights again.
  const auto profile = autolivery_test::vtolProfile();
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{900.0, 400.0, 120.0});
  Mission mission = makeMission(profile, map);
  FakeVehicle vehicle(profile, autolivery_test::kHome, map);
  vehicle.quadchute = true;
  ASSERT_EQ(mission.start({{700.0, 50.0, 0.0}, "farm"}, vehicle.snapshot(), 0.0), "");
  const RunRecord first = autolivery_test::run(mission, vehicle, 900.0);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(first);
  ASSERT_FALSE(mission.result().delivered);
  ASSERT_FALSE(vehicle.saw_fixed_wing);

  vehicle.quadchute = false;  // checked and rebooted
  const double t0 = first.end_time + 10.0;
  ASSERT_EQ(mission.start({{700.0, 50.0, 0.0}, "farm"}, vehicle.snapshot(), t0), "");
  const RunRecord second = autolivery_test::run(
    mission, vehicle, 900.0, [](double, FakeVehicle &, Mission &) {}, t0);
  ASSERT_TRUE(mission.result().success) << dump(second);
  EXPECT_TRUE(second.requested_fw);
  EXPECT_TRUE(vehicle.saw_fixed_wing);
  EXPECT_EQ(vehicle.quadchutes, 1);
}

TEST_F(MissionTest, RefusesDeliveriesItCannotFly)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  const auto snap = vehicle.snapshot();

  auto has = [](const std::string & text, const std::string & word) {
      return text.find(word) != std::string::npos;
    };
  EXPECT_TRUE(has(mission.start({autolivery_test::kInsideHospital, "x"}, snap, 0.0), "blocked"));
  EXPECT_TRUE(has(mission.start({{900.0, 0.0, 0.0}, "x"}, snap, 0.0), "geofence"));
  EXPECT_TRUE(has(mission.start({{55.0, 50.0, 0.0}, "x"}, snap, 0.0), "below the surface"));

  auto low = snap;
  low.battery = 0.4;
  EXPECT_TRUE(has(mission.start({autolivery_test::kPark, "park"}, low, 0.0), "battery"));
  auto armed = snap;
  armed.armed = true;
  EXPECT_TRUE(has(mission.start({autolivery_test::kPark, "park"}, armed, 0.0), "armed"));
  auto empty = snap;
  empty.payload = PayloadState::Released;
  EXPECT_TRUE(has(mission.start({autolivery_test::kPark, "park"}, empty, 0.0), "parcel"));
  auto away = snap;
  away.position = {50.0, -50.0, 0.0};
  EXPECT_TRUE(has(mission.start({autolivery_test::kPark, "park"}, away, 0.0), "depot"));

  auto short_range = profile;
  short_range.max_range = 300.0;
  Mission limited = makeMission(short_range, city);
  EXPECT_TRUE(has(limited.start({autolivery_test::kPark, "park"}, snap, 0.0), "range"));

  EXPECT_EQ(mission.state(), MissionState::Idle);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, snap, 0.0), "");
  EXPECT_TRUE(has(mission.start({autolivery_test::kPark, "park"}, snap, 0.0), "already running"));
}

TEST_F(MissionTest, RejectsInvalidProfilesAndThresholds)
{
  auto bad = autolivery_test::planeProfile();
  bad.dropoff_method = DropoffMethod::Hover;
  EXPECT_THROW(makeMission(bad, city), std::invalid_argument);
  MissionConfig cfg = config();
  cfg.battery_land = 0.5;
  EXPECT_THROW(makeMission(autolivery_test::quadProfile(), city, cfg), std::invalid_argument);
}

TEST_F(MissionTest, LowBatteryOnTheWayOutAbandonsTheDeliveryAndReturns)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound && m.status().leg_remaining < 120.0) {
        v.battery = std::min(v.battery, 0.25);
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().success);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(mission.result().message.find("battery low"), std::string::npos);
  EXPECT_FALSE(visited(r, MissionState::Release));
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 2.0);
  expectNoCollisions(vehicle, *city);
}

TEST_F(MissionTest, CriticalBatteryLandsWhereItIs)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  Vec3 where;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound && m.status().leg_remaining < 100.0 &&
      v.battery > 0.1)
      {
        v.battery = 0.1;
        where = v.position();
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().success);
  EXPECT_NE(mission.result().message.find("critical"), std::string::npos);
  EXPECT_LT(horizontalDistance(vehicle.position(), where), 15.0);
  EXPECT_FALSE(vehicle.armed);
}

TEST_F(MissionTest, PilotTakeoverEndsTheMissionAndSilencesIt)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound && m.status().leg_remaining < 150.0) {
        v.mode = "POSCTL";
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_NE(mission.result().message.find("POSCTL"), std::string::npos);

  // After a takeover the mission must not stream setpoints or ask for modes, ever again.
  auto snap = vehicle.snapshot();
  for (int i = 0; i < 5; ++i) {
    const auto cmd = mission.update(snap, r.end_time + i);
    EXPECT_EQ(cmd.setpoint.kind, Setpoint::Kind::None);
    EXPECT_TRUE(cmd.mode.empty());
    EXPECT_FALSE(cmd.armed.has_value());
    EXPECT_FALSE(cmd.release);
  }
}

TEST_F(MissionTest, AutopilotFailsafeIsTreatedAsATakeover)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Descend) {
        v.mode = px4_mode::kRtl;  // e.g. PX4 datalink-loss failsafe
      }
    });
  EXPECT_EQ(mission.state(), MissionState::Failed) << dump(r);
}

TEST_F(MissionTest, DisarmInFlightFails)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound) {
        v.armed = false;
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_NE(mission.result().message.find("disarmed"), std::string::npos);
}

TEST_F(MissionTest, StuckGripperAbortsAndBringsTheParcelHome)
{
  const auto profile = autolivery_test::quadProfile();
  MissionConfig cfg = config();
  cfg.release_timeout = 10.0;
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.gripper_stuck = true;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 600.0);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().success);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(mission.result().message.find("did not release"), std::string::npos);
  // Retried with feedback, then gave up.
  EXPECT_GE(vehicle.release_commands, 2);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 2.0);
}

TEST_F(MissionTest, RefusedReleaseIsRetriedAndOnlyThenCountsAsDelivered)
{
  // No payload sensor: the delivery is confirmed by time alone. The first release command is
  // refused (as when the autopilot rejects MAV_CMD_DO_GRIPPER); time passing after a command
  // that never worked must not be taken for a delivery.
  const auto profile = autolivery_test::quadProfile();
  MissionConfig cfg = config();
  cfg.payload_feedback = false;
  cfg.release_duration = 2.0;
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.payload_sensor = false;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  int asked = 0;
  double refused_at = -1.0;
  double delivered_at = -1.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0,
    [&](double t, FakeVehicle &, Mission & m) {
      if (delivered_at < 0.0 && m.result().delivered) {
        delivered_at = t;
      }
    },
    [&](double t) {
      if (++asked == 1) {
        refused_at = t;
        return autolivery_test::Gripper::Refuses;
      }
      return autolivery_test::Gripper::Accepts;
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
  EXPECT_EQ(r.releases_commanded, 2);
  EXPECT_EQ(vehicle.release_commands, 1);
  // Retried a retry period after the refusal, then confirmed release_duration after that.
  EXPECT_GE(delivered_at - refused_at, cfg.release_retry_period + cfg.release_duration - 0.2);
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_LT(horizontalDistance(*vehicle.release_position, autolivery_test::kPark), 1.5);
}

TEST_F(MissionTest, ReleaseThatKeepsFailingBringsTheParcelHome)
{
  const auto profile = autolivery_test::quadProfile();
  for (const auto behaviour : {autolivery_test::Gripper::Refuses,
      autolivery_test::Gripper::Silent})
  {
    MissionConfig cfg = config();
    cfg.payload_feedback = false;
    cfg.release_duration = 2.0;
    Mission mission = makeMission(profile, city, cfg);
    FakeVehicle vehicle(profile, autolivery_test::kHome, city);
    vehicle.payload_sensor = false;
    ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
    const RunRecord r = autolivery_test::run(
      mission, vehicle, 600.0, [](double, FakeVehicle &, Mission &) {},
      [&](double) {return behaviour;});
    ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
    EXPECT_FALSE(mission.result().success);
    EXPECT_FALSE(mission.result().delivered) << dump(r);
    EXPECT_NE(mission.result().message.find("did not release"), std::string::npos);
    EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
    EXPECT_EQ(vehicle.release_commands, 0);
    if (behaviour == autolivery_test::Gripper::Refuses) {
      EXPECT_GE(r.releases_commanded, 3);  // kept trying until the release timeout
    }
    EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 2.0);
  }
}

TEST_F(MissionTest, AirDropWhoseReleaseFailsGoesAroundForAnotherPass)
{
  const auto profile = autolivery_test::planeProfile();
  MissionConfig cfg = config();
  cfg.payload_feedback = false;
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.runway_heading = kPi;
  vehicle.payload_sensor = false;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  int asked = 0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 1200.0, [](double, FakeVehicle &, Mission &) {},
    [&](double) {
      return ++asked == 1 ? autolivery_test::Gripper::Refuses : autolivery_test::Gripper::Accepts;
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << dump(r);
  EXPECT_EQ(r.releases_commanded, 2);
  EXPECT_EQ(vehicle.release_commands, 1);
  const bool went_around = std::any_of(
    r.events.begin(), r.events.end(), [](const std::string & e) {
      return e.find("going around") != std::string::npos;
    });
  EXPECT_TRUE(went_around) << dump(r);

  // The second pass has to be a real pass: released where the parcel falls onto the target,
  // not wherever the plane was when it turned back.
  ASSERT_TRUE(vehicle.release_position.has_value());
  const Vec3 p = *vehicle.release_position;
  const Vec3 v = *vehicle.release_velocity;
  const double h = p.z - autolivery_test::kPark.z;
  const double t = (v.z + std::sqrt(v.z * v.z + 2.0 * kGravity * h)) / kGravity;
  const Vec3 impact = p.horizontal() + v.horizontal() * t;
  EXPECT_LT(horizontalDistance(impact, autolivery_test::kPark), 12.0)
    << "impact at (" << impact.x << ", " << impact.y << ")\n" << dump(r);
}

TEST_F(MissionTest, AirDropNeverReleasesFromBeyondTheTarget)
{
  // A plane that only joins the approach line after the release point (here: put on the
  // line 60 m past the target) must hold the parcel and go around, not let it go there.
  const auto profile = autolivery_test::planeProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.runway_heading = kPi;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const Path route = mission.outboundPath();
  ASSERT_GE(route.size(), 3u);
  const Vec3 exit = route.back();
  const Vec3 entry = route[route.size() - 2];
  const Vec3 dir = (exit - entry).horizontal().normalized();
  const Vec3 target{autolivery_test::kPark.x, autolivery_test::kPark.y, entry.z};
  bool moved = false;
  int released_past_target = 0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 1500.0, [&](double, FakeVehicle & v, Mission & m) {
      if (!moved && m.state() == MissionState::Outbound && m.status().leg_remaining < 500.0) {
        v.place(target + dir * 60.0);
        moved = true;
      }
      if (v.release_position && (*v.release_position - target).dot(dir) > 20.0) {
        ++released_past_target;
      }
    });
  ASSERT_TRUE(moved);
  EXPECT_EQ(released_past_target, 0) << dump(r);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << dump(r);
}

TEST_F(MissionTest, VehicleDataDropoutInFlightHoldsThenCarriesOn)
{
  // The link (or the position on it) goes quiet for a moment on the outbound leg. Steering
  // by a position that has stopped updating would fly the vehicle off its route at cruise
  // speed, so it must stop and wait, and pick the route up again when the data is back.
  const auto profile = autolivery_test::quadProfile();
  for (const bool link_lost : {true, false}) {
    Mission mission = makeMission(profile, city);
    FakeVehicle vehicle(profile, autolivery_test::kHome, city);
    ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
    double lost_at = -1.0;
    double speed_when_lost = 0.0;
    double speed_before_return = -1.0;
    const double outage = 2.5;  // shorter than telemetry_timeout
    const RunRecord r = autolivery_test::run(
      mission, vehicle, 600.0, [&](double t, FakeVehicle & v, Mission & m) {
        if (lost_at < 0.0 && m.state() == MissionState::Outbound &&
        m.status().leg_remaining < 150.0)
        {
          lost_at = t;
          speed_when_lost = v.velocity().norm();
          (link_lost ? v.connected : v.position_ok) = false;
        } else if (lost_at >= 0.0 && t - lost_at >= outage && speed_before_return < 0.0) {
          speed_before_return = v.velocity().norm();
          v.connected = true;
          v.position_ok = true;
        }
      });
    ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
    EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << dump(r);
    EXPECT_GT(speed_when_lost, 5.0);
    EXPECT_LT(speed_before_return, 0.5) << "kept flying on stale data";
    expectNoCollisions(vehicle, *city);
  }
}

TEST_F(MissionTest, VehicleDataLostInFlightEndsTheMissionAndGoesQuiet)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  double lost_at = -1.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double t, FakeVehicle & v, Mission & m) {
      if (lost_at < 0.0 && m.state() == MissionState::Outbound &&
      m.status().leg_remaining < 150.0)
      {
        lost_at = t;
        v.connected = false;
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_NE(mission.result().message.find("no link to the vehicle"), std::string::npos);
  EXPECT_NEAR(r.end_time - lost_at, MissionConfig().telemetry_timeout, 0.3);
  // The vehicle was stopped, not flown blind, while the mission waited.
  EXPECT_LT(vehicle.velocity().norm(), 0.5);
  // And from here on the autopilot's own failsafe has it: nothing more from the mission.
  for (int i = 0; i < 5; ++i) {
    const auto cmd = mission.update(vehicle.snapshot(), r.end_time + i);
    EXPECT_EQ(cmd.setpoint.kind, Setpoint::Kind::None);
    EXPECT_TRUE(cmd.mode.empty());
    EXPECT_FALSE(cmd.armed.has_value());
  }
}

TEST_F(MissionTest, ProblemsOnlyVisibleOnceTheVehicleConnectsStopItBeforeArming)
{
  // start() can only check what it can see. A vehicle that was still booting when the
  // delivery was accepted gets the same checks again when its data arrives.
  const auto profile = autolivery_test::quadProfile();
  struct Case
  {
    Vec3 where;
    double battery;
    const char * expected;
  };
  for (const Case & c : {Case{{40.0, -30.0, 0.0}, 1.0, "from the depot"},
      Case{autolivery_test::kHome, 0.4, "battery"}})
  {
    Mission mission = makeMission(profile, city);
    FakeVehicle vehicle(profile, autolivery_test::kHome, city);
    vehicle.connected = false;
    vehicle.place(c.where);
    vehicle.battery = c.battery;
    ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
    bool ever_armed = false;
    const RunRecord r = autolivery_test::run(
      mission, vehicle, 60.0, [&](double t, FakeVehicle & v, Mission &) {
        v.connected = t > 1.0;
        ever_armed = ever_armed || v.armed;
      });
    ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
    EXPECT_NE(mission.result().message.find(c.expected), std::string::npos)
      << mission.result().message;
    EXPECT_FALSE(ever_armed);
  }
}

TEST_F(MissionTest, HighCruiseAltitudeGetsTheTimeItsDescentNeeds)
{
  // 107 m down at 1.5 m/s takes longer than the default descend timeout; that is a long
  // descent, not a vehicle that cannot settle.
  auto profile = autolivery_test::quadProfile();
  profile.cruise_altitude = 110.0;
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 900.0);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << dump(r);
}

TEST_F(MissionTest, CancelInFlightReturnsHome)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  bool cancelled = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double, FakeVehicle &, Mission & m) {
      if (!cancelled && m.state() == MissionState::Outbound && m.status().leg_remaining < 150.0) {
        m.cancel();
        cancelled = true;
      }
    });
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(mission.result().message.find("cancelled"), std::string::npos);
  EXPECT_LT(horizontalDistance(vehicle.position(), autolivery_test::kHome), 2.0);
  expectNoCollisions(vehicle, *city);
}

TEST_F(MissionTest, CancelBeforeTakeoffStandsDown)
{
  const auto profile = autolivery_test::quadProfile();
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 60.0, [](double t, FakeVehicle &, Mission & m) {
      // Still streaming setpoints before the OFFBOARD request.
      if (t > 0.5 && m.state() == MissionState::Preflight) {
        m.cancel();
      }
    });
  EXPECT_EQ(mission.state(), MissionState::Failed) << dump(r);
  // Stand-down keeps asking for disarm while the vehicle sits armed on the pad.
  vehicle.armed = true;
  const auto cmd = mission.update(vehicle.snapshot(), r.end_time);
  ASSERT_TRUE(cmd.armed.has_value());
  EXPECT_FALSE(*cmd.armed);
}

TEST_F(MissionTest, VehicleThatNeverConnectsTimesOut)
{
  const auto profile = autolivery_test::quadProfile();
  MissionConfig cfg = config();
  cfg.preflight_timeout = 5.0;
  Mission mission = makeMission(profile, city, cfg);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  vehicle.connected = false;
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 30.0);
  EXPECT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_NE(mission.result().message.find("not ready"), std::string::npos);
}

TEST_F(MissionTest, AvoidsHeadOnTrafficAndStillDelivers)
{
  const auto profile = autolivery_test::quadProfile();
  AvoidanceParams ap;
  ap.vehicle_radius = profile.body_radius;
  ap.max_horizontal_speed = profile.max_speed;
  VelocityObstacleAvoider avoider(ap);

  // Once the outbound leg starts, an intruder flies down the first leg of the route towards
  // the vehicle at 4 m/s: a guaranteed head-on encounter.
  struct Traffic
  {
    bool active{false};
    double t0{0.0};
    Vec3 start;
    Vec3 velocity;
    double now{0.0};
    DynamicObstacle at() const
    {
      if (!active) {
        return {"intruder", {-1000.0, -1000.0, 40.0}, {}, 1.5};
      }
      return {"intruder", start + velocity * (now - t0), velocity, 1.5};
    }
  } traffic;
  auto avoid = [&](const Vec3 & pos, const Vec3 & desired) {
      return avoider.compute(pos, desired, {traffic.at()}, city.get());
    };

  Mission mission = makeMission(profile, city, config(), avoid);
  FakeVehicle vehicle(profile, autolivery_test::kHome, city);
  ASSERT_EQ(mission.start({autolivery_test::kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const Path & route = mission.outboundPath();
  ASSERT_GE(route.size(), 2u);
  double closest = 1e9;
  bool avoided = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double t, FakeVehicle & v, Mission & m) {
      traffic.now = t;
      if (!traffic.active && m.state() == MissionState::Outbound) {
        traffic.active = true;
        traffic.t0 = t;
        traffic.start = route[1];
        traffic.velocity = (route[0] - route[1]).normalized() * 4.0;
      }
      if (traffic.active) {
        closest = std::min(closest, distance(v.position(), traffic.at().position));
      }
      avoided = avoided || m.status().avoiding;
    });
  ASSERT_TRUE(mission.result().success) << dump(r);
  EXPECT_TRUE(avoided);
  // Required separation is intruder radius + body radius + safety margin; allow a little
  // for the 20 Hz loop and the vehicle's acceleration limit.
  EXPECT_GT(closest, 1.5 + profile.body_radius + ap.safety_margin - 1.0);
  expectNoCollisions(vehicle, *city);
}
