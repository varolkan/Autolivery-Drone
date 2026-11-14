// Weather: what the mission does with a report, and what it does when the air carries the
// vehicle off whatever the report said.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "autolivery_core/mission.hpp"
#include "test_support.hpp"

using namespace autolivery_core;  // NOLINT
using autolivery_test::config;
using autolivery_test::dump;
using autolivery_test::expectNoCollisions;
using autolivery_test::FakeVehicle;
using autolivery_test::kHome;
using autolivery_test::kPark;
using autolivery_test::kShopRoof;
using autolivery_test::makeMission;
using autolivery_test::quadProfile;
using autolivery_test::RunRecord;
using autolivery_test::visited;

namespace
{

Weather wind(const Vec3 & velocity)
{
  Weather w;
  w.wind = velocity;
  return w;
}

// The route length the mission announced when it accepted the delivery.
double announcedRoute(const RunRecord & r)
{
  const std::regex route("route ([0-9]+) m");
  for (const auto & e : r.events) {
    std::smatch m;
    if (std::regex_search(e, m, route)) {
      return std::stod(m[1]);
    }
  }
  return -1.0;
}

double highest(const FakeVehicle & v)
{
  double z = 0.0;
  for (const auto & p : v.trajectory) {
    z = std::max(z, p.z);
  }
  return z;
}

}  // namespace

class WeatherTest : public ::testing::Test
{
protected:
  std::shared_ptr<WorldMap> city = autolivery_test::makeCity();
  VehicleProfile profile = quadProfile();  // limits: wind 8, gusts 12, dry, -10 to 40 C
};

TEST_F(WeatherTest, WindOverTheLimitRefusesTheDeliveryOnTheGround)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);

  vehicle.reported_weather = wind({-9.0, 0.0, 0.0});
  EXPECT_EQ(
    mission.start({kPark, "park"}, vehicle.snapshot(), 0.0),
    "wind of 9.0 m/s is over the test_quad limit of 8.0 m/s");
  EXPECT_EQ(mission.state(), MissionState::Idle);

  // At the limit it goes, and says what wind it is working with.
  vehicle.reported_weather = wind({0.0, 8.0, 0.0});
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const auto events = mission.takeEvents();
  ASSERT_FALSE(events.empty());
  EXPECT_NE(events.front().find("wind 8.0 m/s"), std::string::npos) << events.front();
}

TEST_F(WeatherTest, EveryLimitIsHeldAgainstItsOwnReading)
{
  Weather gusty = wind({3.0, 0.0, 0.0});
  gusty.gust = 12.5;
  Weather wet;
  wet.precipitation = 2.0;
  Weather cold;
  cold.temperature = -15.0;
  Weather hot;
  hot.temperature = 45.0;
  const std::vector<std::pair<Weather, std::string>> cases = {
    {gusty, "gusts of 12.5 m/s are over the test_quad limit of 12.0 m/s"},
    {wet, "precipitation of 2.0 mm/h is over the test_quad limit of 0.1 mm/h"},
    {cold, "temperature of -15.0 C is below the test_quad limit of -10.0 C"},
    {hot, "temperature of 45.0 C is over the test_quad limit of 40.0 C"},
  };
  for (const auto & [weather, refusal] : cases) {
    Mission mission = makeMission(profile, city);
    FakeVehicle vehicle(profile, kHome, city);
    vehicle.reported_weather = weather;
    EXPECT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), refusal);
  }

  // A breezy, damp, cool day inside every limit is a flying day.
  Weather fine = wind({-4.0, 3.0, 0.0});
  fine.gust = 9.0;
  fine.precipitation = 0.05;
  fine.temperature = 4.0;
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = fine;
  EXPECT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
}

TEST_F(WeatherTest, WhatAReportLeavesOutIsNotJudged)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = Weather{};  // a station that is up but has nothing to say
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const auto events = mission.takeEvents();
  EXPECT_NE(events.front().find("wind not reported"), std::string::npos) << events.front();
}

TEST_F(WeatherTest, NoReportFliesUncheckedUnlessOneIsRequired)
{
  FakeVehicle vehicle(profile, kHome, city);
  {
    Mission mission = makeMission(profile, city);
    ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
    const auto events = mission.takeEvents();
    EXPECT_NE(events.front().find("no weather report"), std::string::npos) << events.front();
  }
  MissionConfig cfg = config();
  cfg.require_weather_report = true;
  Mission mission = makeMission(profile, city, cfg);
  EXPECT_NE(
    mission.start({kPark, "park"}, vehicle.snapshot(), 0.0).find("no current weather report"),
    std::string::npos);
  vehicle.reported_weather = wind({2.0, 0.0, 0.0});
  EXPECT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
}

TEST_F(WeatherTest, WindPickingUpInFlightBringsTheParcelHome)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = wind({3.0, 0.0, 0.0});
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  double outbound_since = -1.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double t, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound && outbound_since < 0.0) {
        outbound_since = t;
      }
      if (outbound_since >= 0.0 && t - outbound_since > 5.0) {
        v.reported_weather = wind({11.0, 0.0, 0.0});  // the station now reads a gale
      }
    });

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().success);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(
    mission.result().message.find("wind of 11.0 m/s is over the test_quad limit of 8.0 m/s"),
    std::string::npos) << mission.result().message;
  EXPECT_NE(mission.result().message.find("abandoning delivery"), std::string::npos);
  // It never went down to the dropoff, and the parcel is back at the depot with it.
  EXPECT_FALSE(visited(r, MissionState::Descend)) << dump(r);
  EXPECT_FALSE(visited(r, MissionState::Release)) << dump(r);
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
  EXPECT_EQ(vehicle.release_commands, 0);
  EXPECT_LT(horizontalDistance(vehicle.position(), kHome), 1.5);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  expectNoCollisions(vehicle, *city);
}

TEST_F(WeatherTest, OneBadReadingDoesNotTurnTheVehicleRound)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = wind({3.0, 0.0, 0.0});
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  double outbound_since = -1.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double t, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Outbound && outbound_since < 0.0) {
        outbound_since = t;
      }
      // Over the gust limit for three seconds, less than weather_abort_after, then back.
      const bool spike = outbound_since >= 0.0 && t - outbound_since > 5.0 &&
      t - outbound_since < 8.0;
      Weather w = wind({3.0, 0.0, 0.0});
      w.gust = spike ? 14.0 : 6.0;
      v.reported_weather = w;
    });

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
}

TEST_F(WeatherTest, WeatherTurningBeforeTheMotorsStartStandsTheVehicleDown)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  Weather fine = wind({3.0, 0.0, 0.0});
  fine.precipitation = 0.0;
  vehicle.reported_weather = fine;
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  // The mission is accepted; while the setpoint stream is warming up it starts to rain.
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 60.0, [&](double t, FakeVehicle & v, Mission &) {
      if (t > 0.5) {
        Weather rain = fine;
        rain.precipitation = 3.0;
        v.reported_weather = rain;
      }
    });

  ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_NE(mission.result().message.find("precipitation of 3.0 mm/h"), std::string::npos);
  EXPECT_FALSE(visited(r, MissionState::Takeoff));
  EXPECT_FALSE(vehicle.armed);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_LT(highest(vehicle), 0.1);
}

TEST_F(WeatherTest, LandedAtTheDropoffItSitsOutTheWeatherThenComesHome)
{
  profile.dropoff_method = DropoffMethod::Land;
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = wind({2.0, 0.0, 0.0});
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  // A squall comes through while the vehicle is on the ground at the dropoff, and passes.
  double relaunch_since = -1.0;
  bool waited = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double t, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Relaunch && relaunch_since < 0.0) {
        relaunch_since = t;
      }
      const bool squall = relaunch_since >= 0.0 && t - relaunch_since < 20.0;
      Weather w = wind({squall ? 10.0 : 2.0, 0.0, 0.0});
      v.reported_weather = w;
      if (squall && m.state() == MissionState::Relaunch) {
        waited = true;
        EXPECT_FALSE(v.armed) << "took off into the squall";
      }
    });

  EXPECT_TRUE(waited);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
  EXPECT_LT(horizontalDistance(vehicle.position(), kHome), 1.5);
}

TEST_F(WeatherTest, LandedAtTheDropoffItStaysThereIfTheWeatherDoesNotPass)
{
  profile.dropoff_method = DropoffMethod::Land;
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.reported_weather = wind({2.0, 0.0, 0.0});
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Relaunch) {
        v.reported_weather = wind({10.0, 0.0, 0.0});
      }
    });

  ASSERT_EQ(mission.state(), MissionState::Failed) << dump(r);
  EXPECT_TRUE(mission.result().delivered);  // the parcel was already on the ground
  EXPECT_FALSE(mission.result().success);
  EXPECT_NE(
    mission.result().message.find("staying landed at the dropoff"), std::string::npos) <<
    mission.result().message;
  EXPECT_FALSE(vehicle.armed);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_LT(horizontalDistance(vehicle.position(), kPark), 1.5);
}

TEST_F(WeatherTest, AHeadwindCostsRangeAndATailwindGivesNoneBack)
{
  // How long the route is, from a delivery in still air.
  double route = 0.0;
  {
    Mission mission = makeMission(profile, city);
    FakeVehicle vehicle(profile, kHome, city);
    ASSERT_EQ(mission.start({kShopRoof, "shop roof"}, vehicle.snapshot(), 0.0), "");
    RunRecord r;
    r.events = mission.takeEvents();
    route = announcedRoute(r);
    ASSERT_GT(route, 2.0 * horizontalDistance(kHome, kShopRoof));
  }
  // A battery good for that route and 20 m more.
  profile.max_range = route + 20.0;
  const Vec3 to_shop = (kShopRoof - kHome).horizontal().normalized();
  const Vec3 across{-to_shop.y, to_shop.x, 0.0};
  const auto refusal = [&](const Vec3 & air) {
      Mission mission = makeMission(profile, city);
      FakeVehicle vehicle(profile, kHome, city);
      vehicle.reported_weather = wind(air);
      return mission.start({kShopRoof, "shop roof"}, vehicle.snapshot(), 0.0);
    };

  EXPECT_EQ(refusal({}), "");
  EXPECT_EQ(refusal(to_shop * -1.0), "");  // a light headwind fits in the reserve
  // 7 m/s on the nose going out: the leg takes 12 m/s of airspeed at 5 m/s over the ground.
  const std::string headwind = refusal(to_shop * -7.0);
  EXPECT_NE(headwind.find("of air in this wind"), std::string::npos) << headwind;
  EXPECT_NE(headwind.find("range is"), std::string::npos) << headwind;
  // 3 m/s from behind going out is 3 m/s on the nose coming back. The way back costs more
  // than the reserve, and the push on the way out does not pay for it: the vehicle was in
  // the air just as long.
  EXPECT_NE(refusal(to_shop * 3.0).find("of air in this wind"), std::string::npos);
  // Holding a course across it costs airspeed too.
  EXPECT_NE(refusal(across * 7.0).find("of air in this wind"), std::string::npos);
}

TEST_F(WeatherTest, LegDeadlinesAllowForTheWindInHand)
{
  // 7 m/s on the nose on the way out, for real. The vehicle makes 5 m/s over the ground
  // instead of 8, so the leg takes longer than its still-air time.
  const Vec3 headwind = (kHome - kShopRoof).horizontal().normalized() * 7.0;
  enum class Told {Never, AtTheStart, OnTheClimbOut};
  const auto fly = [&](MissionConfig cfg, Told told, double * outbound_time) {
      Mission mission = makeMission(profile, city, cfg);
      FakeVehicle vehicle(profile, kHome, city);
      vehicle.wind = headwind;
      if (told == Told::AtTheStart) {
        vehicle.reported_weather = wind(headwind);
      }
      EXPECT_EQ(mission.start({kShopRoof, "shop roof"}, vehicle.snapshot(), 0.0), "");
      std::map<MissionState, double> entered;
      const RunRecord r = autolivery_test::run(
        mission, vehicle, 600.0, [&](double t, FakeVehicle & v, Mission & m) {
          entered.emplace(m.state(), t);
          if (told == Told::OnTheClimbOut && m.state() == MissionState::Takeoff) {
            v.reported_weather = wind(headwind);
          }
        });
      if (outbound_time && entered.count(MissionState::Descend)) {
        *outbound_time = entered.at(MissionState::Descend) - entered.at(MissionState::Outbound);
      }
      return mission.result();
    };

  double outbound_time = 0.0;
  ASSERT_TRUE(fly(config(), Told::Never, &outbound_time).success);
  const double length = horizontalDistance(kHome, kShopRoof);
  const double still_air = length / profile.cruise_speed;
  const double in_this_wind = length / (profile.max_speed - 7.0);
  ASSERT_GT(outbound_time, still_air + 5.0);

  // A deadline with no slack to speak of: the leg's expected time plus a margin that puts
  // the real time between what still air and what this wind would allow.
  MissionConfig tight = config();
  tight.leg_timeout_factor = 1.0;
  tight.leg_timeout_margin = outbound_time - 0.5 * (still_air + in_this_wind);

  // Not told about the wind, the mission expects the still-air time and gives up on the leg.
  const MissionResult untold = fly(tight, Told::Never, nullptr);
  EXPECT_FALSE(untold.delivered);
  EXPECT_NE(untold.message.find("outbound leg took too long"), std::string::npos) <<
    untold.message;
  // Told, it expects the slower leg and lets it finish -- whether it heard before the
  // delivery was accepted or only once it was already climbing out.
  const MissionResult told = fly(tight, Told::AtTheStart, nullptr);
  EXPECT_TRUE(told.success) << told.message;
  const MissionResult told_late = fly(tight, Told::OnTheClimbOut, nullptr);
  EXPECT_TRUE(told_late.success) << told_late.message;
}

TEST_F(WeatherTest, BlownOffThePadOnTheWayUpItComesStraightDown)
{
  // Nobody reported it, but it is blowing harder than this vehicle can fly.
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  vehicle.wind = {0.0, -15.0, 0.0};
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 600.0);

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().success);
  EXPECT_NE(
    mission.result().message.find("cannot hold position in the climb-out"),
    std::string::npos) << mission.result().message;
  EXPECT_FALSE(visited(r, MissionState::Outbound)) << dump(r);
  // Caught while still low and close: it never got near cruise altitude, and it is on the
  // ground within a stone's throw of the pad rather than wherever the wind was going.
  EXPECT_LT(highest(vehicle), 0.6 * profile.cruise_altitude);
  EXPECT_LT(horizontalDistance(vehicle.position(), kHome), 100.0);
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
}

TEST_F(WeatherTest, CarriedOffTheDropoffItClimbsOutAndGoesHome)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  // A squall hits as the vehicle starts down over the dropoff, and is over ten seconds later.
  double descend_since = -1.0;
  double furthest = 0.0;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 900.0, [&](double t, FakeVehicle & v, Mission & m) {
      if (m.state() == MissionState::Descend && descend_since < 0.0) {
        descend_since = t;
      }
      const bool squall = descend_since >= 0.0 && t - descend_since < 10.0;
      v.wind = squall ? Vec3{16.0, 0.0, 0.0} : Vec3{};
      if (m.state() == MissionState::Descend) {
        furthest = std::max(furthest, horizontalDistance(v.position(), kPark));
      }
    });

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_FALSE(mission.result().delivered);
  EXPECT_NE(
    mission.result().message.find("cannot hold position over the dropoff"),
    std::string::npos) << mission.result().message;
  EXPECT_FALSE(visited(r, MissionState::Release)) << dump(r);
  // It gave up the descent once it was properly adrift, not a minute later on a timeout.
  EXPECT_GT(furthest, 8.0);
  EXPECT_LT(furthest, 30.0);
  EXPECT_EQ(vehicle.payload(), PayloadState::Attached);
  EXPECT_LT(horizontalDistance(vehicle.position(), kHome), 1.5);
  EXPECT_TRUE(vehicle.onGround());
  expectNoCollisions(vehicle, *city);
}

TEST_F(WeatherTest, AJumpInThePositionEstimateIsFlownOut)
{
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");

  // Half-way up the climb-out the vehicle is suddenly ten metres to one side: an estimator
  // reset, or one hard shove. It flies back; that is not being carried off.
  bool jumped = false;
  const RunRecord r = autolivery_test::run(
    mission, vehicle, 600.0, [&](double, FakeVehicle & v, Mission & m) {
      if (!jumped && m.state() == MissionState::Takeoff && v.position().z > 20.0) {
        v.place(v.position() + Vec3{10.0, 0.0, 0.0});
        jumped = true;
      }
    });

  EXPECT_TRUE(jumped);
  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
}

TEST_F(WeatherTest, DeliversInAStiffBreezeInsideItsLimits)
{
  // 7 m/s from the north-east with gusts to 10: on the nose for most of the way out.
  Mission mission = makeMission(profile, city);
  FakeVehicle vehicle(profile, kHome, city);
  const Vec3 air = Vec3{-1.0, -1.0, 0.0}.normalized() * 7.0;
  vehicle.wind = air;
  vehicle.gust = 3.0;
  Weather report = wind(air);
  report.gust = 10.0;
  vehicle.reported_weather = report;
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 900.0);

  ASSERT_EQ(mission.state(), MissionState::Complete) << dump(r);
  EXPECT_TRUE(mission.result().success) << mission.result().message;
  ASSERT_TRUE(vehicle.release_position.has_value());
  EXPECT_LT(horizontalDistance(*vehicle.release_position, kPark), 1.5);
  EXPECT_LT(horizontalDistance(vehicle.position(), kHome), 1.5);
  expectNoCollisions(vehicle, *city);
}

// The wind as the kinematic vehicle feels it. Every simulated flight in this repository
// stands on this model, so what it does in moving air is pinned down here.
class KinematicWind : public ::testing::Test
{
protected:
  // A vehicle already flying, in OFFBOARD, at `where`.
  FakeVehicle airborne(const VehicleProfile & p, const Vec3 & where)
  {
    FakeVehicle v(p, kHome, city);
    v.armed = true;
    v.mode = px4_mode::kOffboard;
    v.place(where);
    return v;
  }

  // Flies `v` on `setpoint` for `seconds`, calling `each` after every step.
  template<typename Each>
  void fly(FakeVehicle & v, const Setpoint & setpoint, double seconds, Each each)
  {
    const double dt = 0.02;
    for (double t = dt; t <= seconds; t += dt) {
      v.setSetpoint(setpoint, t);
      v.step(t, dt);
      each(t);
    }
  }

  static Setpoint holdAt(const Vec3 & p)
  {
    Setpoint s;
    s.kind = Setpoint::Kind::Position;
    s.position = p;
    return s;
  }

  std::shared_ptr<WorldMap> city = autolivery_test::makeCity();
};

TEST_F(KinematicWind, HoverHoldsItsGroundInWindItCanOutfly)
{
  const Vec3 spot{-100.0, -100.0, 30.0};
  FakeVehicle v = airborne(quadProfile(), spot);
  v.wind = {9.0, 0.0, 0.0};
  v.gust = 2.5;  // gusts to 11.5, still under its 12 m/s
  double furthest = 0.0;
  fly(v, holdAt(spot), 60.0, [&](double) {
      furthest = std::max(furthest, horizontalDistance(v.position(), spot));
    });
  EXPECT_LT(furthest, 1.0);
}

TEST_F(KinematicWind, IsCarriedOffByWindItCannotOutfly)
{
  const Vec3 spot{-100.0, -100.0, 30.0};
  FakeVehicle v = airborne(quadProfile(), spot);
  v.wind = {15.0, 0.0, 0.0};  // 3 m/s more than it has
  fly(v, holdAt(spot), 20.0, [](double) {});
  const Vec3 off = v.position() - spot;
  EXPECT_NEAR(off.x, 3.0 * 20.0, 6.0);  // downwind at the difference
  EXPECT_NEAR(off.y, 0.0, 0.5);
  EXPECT_NEAR(v.velocity().x, 3.0, 0.2);
}

TEST_F(KinematicWind, GustsSwingAboutTheMeanAndNeverTurnTheWindRound)
{
  const Vec3 spot{-100.0, -100.0, 30.0};
  FakeVehicle v = airborne(quadProfile(), spot);
  v.wind = {0.0, -6.0, 0.0};
  v.gust = 4.0;
  double sum = 0.0;
  double strongest = 0.0;
  double weakest = 1e9;
  int samples = 0;
  fly(v, holdAt(spot), 333.0, [&](double) {
      const double speed = -v.airVelocity().y;
      EXPECT_NEAR(v.airVelocity().x, 0.0, 1e-9);
      sum += speed;
      strongest = std::max(strongest, speed);
      weakest = std::min(weakest, speed);
      ++samples;
    });
  EXPECT_NEAR(sum / samples, 6.0, 0.1);   // the mean wind is the mean
  EXPECT_NEAR(strongest, 10.0, 0.2);      // and the strongest gust is mean + gust
  EXPECT_GT(weakest, 1.8);

  // Gusts bigger than the wind itself die away to calm in the lulls; they never blow back.
  v.wind = {0.0, -2.0, 0.0};
  v.gust = 5.0;
  weakest = 1e9;
  fly(v, holdAt(spot), 120.0, [&](double) {weakest = std::min(weakest, -v.airVelocity().y);});
  EXPECT_NEAR(weakest, 0.0, 1e-9);
}

TEST_F(KinematicWind, PlaneCrabsAlongItsCourseAndPaysInGroundSpeed)
{
  const auto profile = autolivery_test::planeProfile();  // 15 m/s airspeed
  const Vec3 start{-250.0, -200.0, 100.0};
  FakeVehicle v = airborne(profile, start);
  v.wind = {0.0, -6.0, 0.0};  // straight across an eastbound course
  const Vec3 target = start + Vec3{5000.0, 0.0, 0.0};
  double worst_off_track = 0.0;
  fly(v, holdAt(target), 30.0, [&](double t) {
      if (t > 10.0) {  // once it has turned onto its heading
        worst_off_track = std::max(worst_off_track, std::abs(v.position().y - start.y));
      }
    });
  EXPECT_LT(worst_off_track, 12.0);
  EXPECT_NEAR(v.velocity().horizontalNorm(), std::sqrt(15.0 * 15.0 - 6.0 * 6.0), 0.3);
  EXPECT_NEAR(v.velocity().y, 0.0, 0.5);
  // The nose points upwind of where it is going.
  EXPECT_GT(v.heading(), 0.2);

  // Straight into it, the ground speed is what the wind leaves.
  FakeVehicle upwind = airborne(profile, start);
  upwind.wind = {-6.0, 0.0, 0.0};
  fly(upwind, holdAt(target), 30.0, [](double) {});
  EXPECT_NEAR(upwind.velocity().x, 9.0, 0.3);
}

TEST_F(WeatherTest, PlaneDropsOnTargetInACrosswind)
{
  const auto plane = autolivery_test::planeProfile();
  Mission mission = makeMission(plane, city);
  FakeVehicle vehicle(plane, kHome, city);
  // Across the run-in from the depot to the park, 6 m/s with gusts to 8.
  const Vec3 run_in = (kPark - kHome).horizontal().normalized();
  const Vec3 across{-run_in.y, run_in.x, 0.0};
  const Vec3 air = across * 6.0;
  vehicle.wind = air;
  vehicle.gust = 2.0;
  vehicle.reported_weather = wind(air);
  ASSERT_EQ(mission.start({kPark, "park"}, vehicle.snapshot(), 0.0), "");
  const RunRecord r = autolivery_test::run(mission, vehicle, 1200.0);

  ASSERT_TRUE(mission.result().delivered) << dump(r);
  ASSERT_TRUE(vehicle.release_position.has_value());
  // Where a drag-free parcel let go there, at that ground velocity, comes down.
  const Vec3 p = *vehicle.release_position;
  const Vec3 v = *vehicle.release_velocity;
  const double h = p.z - kPark.z;
  const double t = (v.z + std::sqrt(v.z * v.z + 2.0 * kGravity * h)) / kGravity;
  const Vec3 impact = p.horizontal() + v.horizontal() * t;
  EXPECT_LT(horizontalDistance(impact, kPark), 12.0)
    << "impact at (" << impact.x << ", " << impact.y << ")\n" << dump(r);
}
