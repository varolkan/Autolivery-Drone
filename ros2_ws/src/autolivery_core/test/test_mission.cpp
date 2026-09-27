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
using autolivery_test::FakeVehicle;
using autolivery_test::RunRecord;

namespace
{

MissionConfig config()
{
  MissionConfig c;
  c.home = autolivery_test::kHome;
  c.payload_feedback = true;
  return c;
}

Mission makeMission(
  const VehicleProfile & profile, const std::shared_ptr<WorldMap> & map,
  MissionConfig cfg = config(), AvoidFn avoid = nullptr)
{
  return Mission(
    profile, cfg, map, autolivery_test::makePlanner(map, profile.planning_clearance),
    std::move(avoid));
}

bool visited(const RunRecord & r, MissionState s)
{
  return std::find(r.states.begin(), r.states.end(), s) != r.states.end();
}

bool visitedInOrder(const RunRecord & r, const std::vector<MissionState> & expected)
{
  auto it = r.states.begin();
  for (auto s : expected) {
    it = std::find(it, r.states.end(), s);
    if (it == r.states.end()) {
      return false;
    }
  }
  return true;
}

std::string dump(const RunRecord & r)
{
  std::string out;
  for (const auto & e : r.events) {
    out += "  " + e + "\n";
  }
  return out;
}

// No point of the flight may be inside a building, a tree or a no-fly zone.
void expectNoCollisions(const FakeVehicle & v, const WorldMap & map)
{
  for (const auto & p : v.trajectory) {
    if (p.z - map.groundHeight(p.x, p.y) < 0.3) {
      continue;  // on the ground (depot pad, dropoff surface)
    }
    ASSERT_GT(map.distanceToObstacles(p), 0.3)
      << "flew into an obstacle at (" << p.x << ", " << p.y << ", " << p.z << ")";
  }
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
