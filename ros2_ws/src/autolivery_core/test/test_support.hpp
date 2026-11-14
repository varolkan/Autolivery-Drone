// Shared fixtures for the core tests: a small city, test profiles, a planner, and a loop that
// steps a Mission against a KinematicVehicle (see kinematic_vehicle.hpp for what it models).

#ifndef TEST_SUPPORT_HPP_
#define TEST_SUPPORT_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autolivery_core/astar.hpp"
#include "autolivery_core/kinematic_vehicle.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_core/path_smoother.hpp"
#include "autolivery_core/voxel_grid.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_test
{

using namespace autolivery_core;  // NOLINT

// 700 m x 700 m test city around a depot at the origin.
//   tower     80 m tall, directly between the depot and the park: must be flown around
//   shop      15 m tall with a rooftop dropoff
//   hospital  no-fly zone south-east of the tower
//   tree      a cylinder next to the park dropoff
inline std::shared_ptr<WorldMap> makeCity()
{
  auto map = std::make_shared<WorldMap>(Vec3{-300.0, -300.0, 0.0}, Vec3{400.0, 400.0, 120.0});
  map->addBuilding({"tower", {40.0, 20.0, 0.0}, {70.0, 80.0, 80.0}});
  map->addBuilding({"shop", {80.0, -50.0, 0.0}, {100.0, -30.0, 15.0}});
  map->addNoFlyZone({"hospital", {120.0, -140.0, 0.0}, {170.0, -90.0, 120.0}});
  map->addCylinder({"tree", {140.0, 95.0, 0.0}, 4.0, 14.0});
  return map;
}

inline const Vec3 kHome{0.0, 0.0, 0.0};
inline const Vec3 kPark{150.0, 110.0, 0.0};
inline const Vec3 kShopRoof{90.0, -40.0, 15.0};
inline const Vec3 kInsideHospital{145.0, -115.0, 0.0};

inline VehicleProfile quadProfile()
{
  VehicleProfile p;
  p.name = "test_quad";
  p.cruise_speed = 8.0;
  p.max_speed = 12.0;
  p.cruise_altitude = 40.0;
  p.planning_clearance = 4.0;
  p.release_altitude = 3.0;
  return p;
}

inline VehicleProfile planeProfile()
{
  VehicleProfile p;
  p.name = "test_plane";
  p.vehicle_class = VehicleClass::FixedWing;
  p.dropoff_method = DropoffMethod::AirDrop;
  p.cruise_speed = 15.0;
  p.max_speed = 20.0;
  p.min_speed = 11.0;
  p.cruise_altitude = 60.0;
  p.takeoff_altitude = 25.0;
  p.turn_radius = 35.0;
  p.acceptance_radius = 40.0;
  p.planning_clearance = 8.0;
  p.release_altitude = 30.0;
  p.max_range = 20000.0;
  return p;
}

inline VehicleProfile vtolProfile()
{
  VehicleProfile p;
  p.name = "test_vtol";
  p.vehicle_class = VehicleClass::Vtol;
  p.dropoff_method = DropoffMethod::Hover;
  p.cruise_speed = 15.0;
  p.max_speed = 20.0;
  p.cruise_altitude = 60.0;
  p.turn_radius = 35.0;
  p.acceptance_radius = 4.0;
  p.planning_clearance = 8.0;
  p.release_altitude = 3.0;
  p.max_range = 20000.0;
  return p;
}

inline PlanFn makePlanner(const std::shared_ptr<const WorldMap> & map, double clearance)
{
  auto grid = std::make_shared<VoxelGrid>(*map, 2.0, clearance);
  auto astar = std::make_shared<AStarPlanner>(grid);
  return [map, astar, clearance](const Vec3 & from, const Vec3 & to) {
           PlanResult r = astar->plan(from, to);
           if (r.success) {
             r.path = shortcutPath(r.path, *map, clearance);
           }
           return r;
         };
}

// The kinematic PX4 stand-in lives in the library so the dry-run FCU node can use it too.
using FakeVehicle = KinematicVehicle;

struct RunRecord
{
  std::vector<MissionState> states;  // distinct states in the order visited
  std::vector<std::string> events;
  bool requested_fw{false};
  int releases_commanded{0};
  double end_time{0.0};
};

// Steps mission and vehicle together at 20 Hz until the mission finishes or time runs out.
// `hook` runs before every tick and can change the vehicle (drain the battery, take over).
// `gripper` stands in for the payload node: it is asked about every release the mission
// commands and says what became of it. Only an accepted command reaches the vehicle.
enum class Gripper
{
  Accepts,  // acknowledged, as the payload node does once the autopilot took the command
  Refuses,  // answered with a failure (autopilot rejected it, service down)
  Silent,   // no answer at all
};

template<typename Hook, typename GripperFn>
RunRecord run(
  Mission & mission, FakeVehicle & vehicle, double max_time, Hook hook, GripperFn gripper,
  double t0 = 0.0)
{
  RunRecord rec;
  const double dt = 0.05;
  double t = t0;
  rec.states.push_back(mission.state());
  while (t < t0 + max_time && !mission.finished()) {
    hook(t, vehicle, mission);
    MissionCommand cmd = mission.update(vehicle.snapshot(), t);
    if (cmd.vtol == VtolRequest::Fw) {
      rec.requested_fw = true;
    }
    if (cmd.release) {
      ++rec.releases_commanded;
      switch (gripper(t)) {
        case Gripper::Accepts:
          mission.releaseAccepted();
          break;
        case Gripper::Refuses:
          cmd.release = false;
          mission.releaseFailed("refused by the autopilot");
          break;
        case Gripper::Silent:
          cmd.release = false;
          break;
      }
    }
    vehicle.step(cmd, t, dt);
    if (rec.states.back() != mission.state()) {
      rec.states.push_back(mission.state());
    }
    for (auto & e : mission.takeEvents()) {
      rec.events.push_back(std::move(e));
    }
    t += dt;
  }
  rec.end_time = t;
  return rec;
}

template<typename Hook>
RunRecord run(Mission & mission, FakeVehicle & vehicle, double max_time, Hook hook, double t0 = 0.0)
{
  return run(mission, vehicle, max_time, hook, [](double) {return Gripper::Accepts;}, t0);
}

inline RunRecord run(Mission & mission, FakeVehicle & vehicle, double max_time)
{
  return run(mission, vehicle, max_time, [](double, FakeVehicle &, Mission &) {});
}

inline MissionConfig config()
{
  MissionConfig c;
  c.home = kHome;
  c.payload_feedback = true;
  return c;
}

inline Mission makeMission(
  const VehicleProfile & profile, const std::shared_ptr<WorldMap> & map,
  MissionConfig cfg = config(), AvoidFn avoid = nullptr)
{
  return Mission(
    profile, cfg, map, makePlanner(map, profile.planning_clearance), std::move(avoid));
}

inline bool visited(const RunRecord & r, MissionState s)
{
  return std::find(r.states.begin(), r.states.end(), s) != r.states.end();
}

inline bool visitedInOrder(const RunRecord & r, const std::vector<MissionState> & expected)
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

inline std::string dump(const RunRecord & r)
{
  std::string out;
  for (const auto & e : r.events) {
    out += "  " + e + "\n";
  }
  return out;
}

// No point of the flight may be inside a building, a tree or a no-fly zone.
inline void expectNoCollisions(const FakeVehicle & v, const WorldMap & map)
{
  for (const auto & p : v.trajectory) {
    if (p.z - map.groundHeight(p.x, p.y) < 0.3) {
      continue;  // on the ground (depot pad, dropoff surface)
    }
    ASSERT_GT(map.distanceToObstacles(p), 0.3)
      << "flew into an obstacle at (" << p.x << ", " << p.y << ", " << p.z << ")";
  }
}

}  // namespace autolivery_test

#endif  // TEST_SUPPORT_HPP_
