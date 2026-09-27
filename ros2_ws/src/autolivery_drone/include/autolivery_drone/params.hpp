// Declares and reads the core library's configuration structs as ROS parameters.
//
// Parameters are declared read-only: a vehicle profile or gripper mapping that changes
// under a flying mission is a hazard, not a feature. Change them in the YAML and restart.
//
// Layout (see config/vehicles/*.yaml and config/grippers/*.yaml):
//   vehicle.*     VehicleProfile
//   mission.*     MissionConfig (everything except the depot, which comes from the map)
//   gripper.*     GripperConfig
//   planner.*     global planner selection and tuning
//   avoidance.*   velocity-obstacle avoidance tuning

#ifndef AUTOLIVERY_DRONE__PARAMS_HPP_
#define AUTOLIVERY_DRONE__PARAMS_HPP_

#include <string>

#include "autolivery_core/astar.hpp"
#include "autolivery_core/gripper.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_core/rrt_star.hpp"
#include "autolivery_core/vehicle_profile.hpp"
#include "autolivery_core/velocity_obstacle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolivery_drone
{

struct PlannerSettings
{
  std::string algorithm{"astar"};  // astar | rrt_star
  double resolution{2.0};
  double max_segment{0.0};         // densify legs longer than this; 0 = off
  autolivery_core::AStarParams astar;
  autolivery_core::RrtStarParams rrt;
};

struct AvoidanceSettings
{
  bool enabled{true};
  double obstacle_timeout{1.0};  // s; obstacles from a silent source are dropped after this
  autolivery_core::AvoidanceParams params;
};

autolivery_core::VehicleProfile declareVehicleProfile(rclcpp::Node & node);
autolivery_core::MissionConfig declareMissionConfig(rclcpp::Node & node);
autolivery_core::GripperConfig declareGripperConfig(rclcpp::Node & node);
PlannerSettings declarePlannerSettings(rclcpp::Node & node);
AvoidanceSettings declareAvoidanceSettings(
  rclcpp::Node & node, const autolivery_core::VehicleProfile & profile);

// Path of a file in this package's share directory, or `value` unchanged if it is already
// an absolute path. Lets launch files and YAML refer to "maps/urban_delivery.yaml".
std::string resolvePackagePath(const std::string & value);

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__PARAMS_HPP_
