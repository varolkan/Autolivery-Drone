// Every shipped vehicle profile flies a whole delivery in the shipped city against the
// kinematic PX4 stand-in, in simulated time: the same Mission, planner and avoidance the node
// runs, with the real map and configs. Catches a profile/map combination that plans fine but
// cannot actually be flown (a turn radius that never closes on a waypoint, an approach the
// timeouts do not allow for) before it reaches the simulator or an airframe.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include "autolivery_core/kinematic_vehicle.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/route_planner.hpp"

using autolivery_core::KinematicVehicle;
using autolivery_core::Mission;
using autolivery_core::MissionConfig;
using autolivery_core::MissionState;
using autolivery_core::Vec3;
using autolivery_core::VehicleProfile;

namespace
{

const char kSource[] = AUTOLIVERY_SOURCE_DIR;

std::string sourcePath(const std::string & relative) {return std::string(kSource) + relative;}

VehicleProfile loadProfile(const std::string & name, std::string * depot)
{
  const YAML::Node params =
    YAML::LoadFile(sourcePath("/config/vehicles/" + name + ".yaml"))["/**"]["ros__parameters"];
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
  p.max_range = v["max_range"].as<double>();
  *depot = params["depot"] ? params["depot"].as<std::string>() : "main";
  return p;
}

struct Case
{
  std::string vehicle;
  std::string location;
};

class ProfileMission : public ::testing::TestWithParam<Case> {};

}  // namespace

TEST_P(ProfileMission, DeliversAndComesHome)
{
  const Case c = GetParam();
  const autolivery_drone::CityMap city =
    autolivery_drone::loadCityMap(sourcePath("/maps/urban_delivery.yaml"));
  std::string depot;
  const VehicleProfile profile = loadProfile(c.vehicle, &depot);
  autolivery_drone::RoutePlanner planner(city.world, profile.planning_clearance, {});

  MissionConfig cfg;
  cfg.home = city.depots.at(depot);
  cfg.payload_feedback = true;
  cfg.fw_mission_landing = city.runways.count(depot) > 0;
  Mission mission(profile, cfg, city.world, planner.asPlanFn());

  KinematicVehicle vehicle(profile, cfg.home, city.world);
  vehicle.fixed_wing_rtl_lands = cfg.fw_mission_landing;
  ASSERT_EQ(
    mission.start({city.locations.at(c.location), c.location}, vehicle.snapshot(), 0.0),
    "");

  std::string log;
  double t = 0.0;
  const double dt = 0.05;
  while (t < 1800.0 && !mission.finished()) {
    vehicle.step(mission.update(vehicle.snapshot(), t), t, dt);
    for (const auto & e : mission.takeEvents()) {
      log += "  " + std::to_string(static_cast<int>(t)) + " s: " + e + "\n";
    }
    t += dt;
  }
  ASSERT_EQ(mission.state(), MissionState::Complete) << c.vehicle << " -> " << c.location << "\n" <<
    log;
  EXPECT_TRUE(mission.result().success) << mission.result().message << "\n" << log;
  EXPECT_TRUE(vehicle.onGround());
  EXPECT_FALSE(vehicle.armed);
  for (const auto & p : vehicle.trajectory) {
    if (p.z - city.world->groundHeight(p.x, p.y) > 0.3) {
      ASSERT_GT(city.world->distanceToObstacles(p), 0.3) <<
        "flew into an obstacle at (" << p.x << ", " << p.y << ", " << p.z << ")\n" << log;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  ShippedProfiles, ProfileMission, ::testing::Values(
    Case{"quadcopter", "rooftop_cafe"},
    Case{"quadcopter", "hotel_roof"},
    Case{"hexacopter", "park"},
    Case{"octocopter", "house_backyard"},
    Case{"coaxial_octocopter", "warehouse_dock"},
    Case{"helicopter", "park"},
    Case{"vtol", "farm"},
    Case{"fixed_wing", "farm"}),
  [](const ::testing::TestParamInfo<Case> & info) {
    return info.param.vehicle + "_to_" + info.param.location;
  });
