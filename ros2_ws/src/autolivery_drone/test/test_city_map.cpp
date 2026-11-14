// The shipped city map and vehicle profiles, checked the way the mission node will use them:
// every profile validates, and every named location is a delivery the mission accepts.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "autolivery_core/mission.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/route_planner.hpp"

using autolivery_core::DeliveryRequest;
using autolivery_core::Mission;
using autolivery_core::MissionConfig;
using autolivery_core::Vec3;
using autolivery_core::VehicleProfile;
using autolivery_core::VehicleSnapshot;
using autolivery_drone::CityMap;
using autolivery_drone::loadCityMap;

namespace
{

const char kSource[] = AUTOLIVERY_SOURCE_DIR;

std::string sourcePath(const std::string & relative) {return std::string(kSource) + relative;}

// Reads a vehicle profile YAML the same way ROS would hand it to declareVehicleProfile.
VehicleProfile loadProfile(const std::string & file, std::string * depot = nullptr)
{
  const YAML::Node params = YAML::LoadFile(file)["/**"]["ros__parameters"];
  const YAML::Node v = params["vehicle"];
  VehicleProfile p;
  p.name = v["name"].as<std::string>();
  p.vehicle_class = autolivery_core::parseVehicleClass(v["class"].as<std::string>());
  p.dropoff_method = autolivery_core::parseDropoffMethod(v["dropoff_method"].as<std::string>());
  p.cruise_speed = v["cruise_speed"].as<double>();
  p.max_speed = v["max_speed"].as<double>();
  p.min_speed = v["min_speed"].as<double>();
  p.max_climb_rate = v["max_climb_rate"].as<double>();
  p.max_descent_rate = v["max_descent_rate"].as<double>();
  p.max_acceleration = v["max_acceleration"].as<double>();
  p.cruise_altitude = v["cruise_altitude"].as<double>();
  p.takeoff_altitude = v["takeoff_altitude"].as<double>();
  p.body_radius = v["body_radius"].as<double>();
  p.planning_clearance = v["planning_clearance"].as<double>();
  p.acceptance_radius = v["acceptance_radius"].as<double>();
  p.turn_radius = v["turn_radius"].as<double>();
  p.release_altitude = v["release_altitude"].as<double>();
  p.max_payload_kg = v["max_payload_kg"].as<double>();
  p.max_range = v["max_range"].as<double>();
  p.px4_airframe = v["px4_airframe"].as<int>();
  if (depot != nullptr) {
    *depot = params["depot"] ? params["depot"].as<std::string>() : "main";
  }
  return p;
}

VehicleSnapshot readyAt(const Vec3 & p)
{
  VehicleSnapshot s;
  s.connected = true;
  s.position_valid = true;
  s.position = p;
  s.battery = 1.0;
  s.payload = autolivery_core::PayloadState::Attached;
  s.mode = "AUTO.LOITER";
  return s;
}

std::string writeTemp(const std::string & name, const std::string & text)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream(path) << text;
  return path.string();
}

}  // namespace

TEST(CityMap, ShippedMapLoadsAndResolvesSurfaces)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  EXPECT_EQ(city.name, "urban_delivery");
  EXPECT_TRUE(city.world->validate().empty());
  EXPECT_EQ(city.depots.count("main"), 1u);
  EXPECT_EQ(city.depots.count("airstrip"), 1u);
  EXPECT_DOUBLE_EQ(city.locations.at("rooftop_cafe").z, 18.0);
  EXPECT_DOUBLE_EQ(city.locations.at("hotel_roof").z, 55.0);
  EXPECT_DOUBLE_EQ(city.locations.at("park").z, 0.0);
  EXPECT_GE(city.traffic.size(), 1u);
  EXPECT_NEAR(city.origin.latitude, 47.397971, 1e-9);
}

TEST(CityMap, EveryShippedProfileIsValid)
{
  int count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(sourcePath("/config/vehicles"))) {
    const auto profile = loadProfile(entry.path().string());
    const auto problems = autolivery_core::validate(profile);
    std::string text;
    for (const auto & p : problems) {
      text += "\n  " + p;
    }
    EXPECT_TRUE(problems.empty()) << entry.path().filename() << ":" << text;
    ++count;
  }
  EXPECT_GE(count, 6);
}

TEST(CityMap, QuadcopterCanDeliverToEveryNamedLocation)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  const auto profile = loadProfile(sourcePath("/config/vehicles/quadcopter.yaml"));
  autolivery_drone::RoutePlanner planner(city.world, profile.planning_clearance, {});
  MissionConfig cfg;
  cfg.home = city.depots.at("main");
  for (const auto & [name, where] : city.locations) {
    if (name == "farm") {
      continue;  // open-country leg for the long-range airframes
    }
    Mission mission(profile, cfg, city.world, planner.asPlanFn());
    EXPECT_EQ(mission.start({where, name}, readyAt(cfg.home), 0.0), "") << name;
  }
}

TEST(CityMap, EveryProfileCanServeSomewhere)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  for (const auto & entry : std::filesystem::directory_iterator(sourcePath("/config/vehicles"))) {
    std::string depot;
    const auto profile = loadProfile(entry.path().string(), &depot);
    ASSERT_EQ(city.depots.count(depot), 1u) << entry.path().filename();
    autolivery_drone::RoutePlanner planner(city.world, profile.planning_clearance, {});
    MissionConfig cfg;
    cfg.home = city.depots.at(depot);
    int served = 0;
    std::string refusals;
    for (const auto & [name, where] : city.locations) {
      Mission mission(profile, cfg, city.world, planner.asPlanFn());
      const auto error = mission.start({where, name}, readyAt(cfg.home), 0.0);
      if (error.empty()) {
        ++served;
      } else {
        refusals += "\n  " + name + ": " + error;
      }
    }
    EXPECT_GT(served, 0) << profile.name << " cannot serve any location:" << refusals;
  }
}

TEST(CityMap, NoFlyZoneDropoffIsRefused)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  const auto profile = loadProfile(sourcePath("/config/vehicles/quadcopter.yaml"));
  autolivery_drone::RoutePlanner planner(city.world, profile.planning_clearance, {});
  MissionConfig cfg;
  cfg.home = city.depots.at("main");
  Mission mission(profile, cfg, city.world, planner.asPlanFn());
  const auto error = mission.start({{170.0, -25.0, 0.0}, "helipad"}, readyAt(cfg.home), 0.0);
  EXPECT_NE(error.find("blocked"), std::string::npos) << error;
}

TEST(CityMap, BothPlannersRouteAcrossTheCity)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  autolivery_drone::PlannerSettings settings;
  settings.rrt.seed = 3;
  settings.rrt.max_planning_time = 5.0;
  autolivery_drone::RoutePlanner planner(city.world, 4.0, settings);
  const Vec3 from{0.0, 0.0, 40.0};
  const Vec3 to{215.0, 78.0, 40.0};  // behind the hotel from the depot
  for (const std::string algorithm : {"astar", "rrt_star"}) {
    const auto r = planner.plan(from, to, algorithm);
    ASSERT_TRUE(r.success) << algorithm << ": " << r.message;
    for (std::size_t i = 1; i < r.path.size(); ++i) {
      EXPECT_TRUE(city.world->segmentClear(r.path[i - 1], r.path[i], 4.0)) << algorithm;
    }
  }
  EXPECT_FALSE(planner.plan(from, to, "dijkstra").success);
}

TEST(CityMap, ConfiguredRrtStarFallsBackToAStar)
{
  // One sample is not enough for RRT* to get round the hotel. As the mission's planner that
  // must still produce a route (from A*); asked for by name it reports its own failure.
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  autolivery_drone::PlannerSettings settings;
  settings.algorithm = "rrt_star";
  settings.rrt.seed = 3;
  settings.rrt.max_iterations = 1;
  autolivery_drone::RoutePlanner planner(city.world, 4.0, settings);
  const Vec3 from{0.0, 0.0, 40.0};
  const Vec3 to{215.0, 78.0, 40.0};

  const auto named = planner.plan(from, to, "rrt_star");
  EXPECT_FALSE(named.success);

  const auto r = planner.plan(from, to);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_NE(r.message.find("by astar"), std::string::npos) << r.message;
  for (std::size_t i = 1; i < r.path.size(); ++i) {
    EXPECT_TRUE(city.world->segmentClear(r.path[i - 1], r.path[i], 4.0));
  }
  const auto fn = planner.asPlanFn();
  EXPECT_TRUE(fn(from, to).success);
}

TEST(CityMap, RrtStarPlansFromInsideAClearanceMargin)
{
  // 2.5 m off the office tower's west wall, inside the 4 m planning clearance: where a
  // vehicle ends up after dodging traffic. RRT* alone can grow nothing from there.
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  autolivery_drone::PlannerSettings settings;
  settings.rrt.seed = 3;
  autolivery_drone::RoutePlanner planner(city.world, 4.0, settings);
  const Vec3 from{37.5, 50.0, 40.0};
  const Vec3 to{0.0, 0.0, 40.0};
  ASSERT_TRUE(city.world->isBlocked(from, 4.0));
  ASSERT_FALSE(city.world->isBlocked(from, 0.6));

  const auto r = planner.plan(from, to, "rrt_star");
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_NEAR(autolivery_core::distance(r.path.front(), from), 0.0, 1e-9);
  EXPECT_NEAR(autolivery_core::distance(r.path.back(), to), 0.0, 1e-9);
  // The first hop leaves the margin (and stays off the wall itself); the rest is clear.
  EXPECT_TRUE(city.world->segmentClear(r.path[0], r.path[1], 0.6));
  EXPECT_LT(autolivery_core::distance(r.path[0], r.path[1]), 10.0);
  for (std::size_t i = 2; i < r.path.size(); ++i) {
    EXPECT_TRUE(city.world->segmentClear(r.path[i - 1], r.path[i], 4.0));
  }
}

TEST(CityMap, BadFilesFailLoudly)
{
  EXPECT_THROW(loadCityMap("/nonexistent/map.yaml"), std::runtime_error);

  const auto no_fence = writeTemp("autolivery_no_fence.yaml", "name: x\ndepots: {main: [0, 0]}\n");
  EXPECT_THROW(loadCityMap(no_fence), std::runtime_error);

  const auto inside = writeTemp(
    "autolivery_inside.yaml",
    "geofence: {min: [-10, -10, 0], max: [10, 10, 10]}\n"
    "depots: {main: [0, 0]}\n"
    "buildings: [{name: b, min: [2, 2, 0], max: [5, 5, 5]}]\n"
    "locations: {bad: [3, 3, 1]}\n");
  try {
    loadCityMap(inside);
    FAIL() << "a location inside a building was accepted";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("bad"), std::string::npos) << e.what();
  }

  const auto inverted = writeTemp(
    "autolivery_inverted.yaml",
    "geofence: {min: [-10, -10, 0], max: [10, 10, 10]}\n"
    "depots: {main: [0, 0]}\n"
    "buildings: [{name: upside_down, min: [5, 5, 5], max: [2, 2, 0]}]\n");
  EXPECT_THROW(loadCityMap(inverted), std::runtime_error);
}

TEST(CityMap, ShippedRunwayIsFlyable)
{
  const CityMap city = loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  ASSERT_EQ(city.runways.count("airstrip"), 1u);
  const auto & rw = city.runways.at("airstrip");
  // The PX4 rc_cessna airframe lands at FW_LND_ANG 8 deg; PX4 rejects steeper mission landings.
  EXPECT_LT(rw.glide_slope_deg, 8.0);
  EXPECT_GT(rw.glide_slope_deg, 3.0);
  EXPECT_NEAR(rw.direction.norm(), 1.0, 1e-9);
  EXPECT_TRUE(city.world->inBounds(rw.approach));
  EXPECT_FALSE(city.world->isBlocked(rw.approach, rw.loiter_radius));
  // The fixed-wing profile flies from this depot.
  std::string depot;
  loadProfile(sourcePath("/config/vehicles/fixed_wing.yaml"), &depot);
  EXPECT_EQ(depot, "airstrip");
}

TEST(CityMap, GeodeticConversion)
{
  autolivery_drone::GeoOrigin o{47.0, 8.0, 400.0};
  const Vec3 north = autolivery_drone::geodeticToWorld(o, 47.001, 8.0, 410.0);
  EXPECT_NEAR(north.y, 111.3, 0.2);
  EXPECT_NEAR(north.x, 0.0, 1e-9);
  EXPECT_NEAR(north.z, 10.0, 1e-9);
  const Vec3 east = autolivery_drone::geodeticToWorld(o, 47.0, 8.001, 400.0);
  EXPECT_NEAR(east.x, 111.3 * std::cos(47.0 * M_PI / 180.0), 0.2);
  for (const Vec3 p : {Vec3{-450.0, 250.0, 30.0}, Vec3{120.0, -80.0, 5.0}}) {
    const Vec3 geo = autolivery_drone::worldToGeodetic(o, p);
    const Vec3 back = autolivery_drone::geodeticToWorld(o, geo.x, geo.y, geo.z);
    EXPECT_NEAR(back.x, p.x, 1e-6);
    EXPECT_NEAR(back.y, p.y, 1e-6);
    EXPECT_NEAR(back.z, p.z, 1e-9);
  }
}
