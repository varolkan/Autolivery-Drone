#include "autolivery_drone/params.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace autolivery_drone
{

namespace
{

template<typename T>
T declare(
  rclcpp::Node & node, const std::string & name, const T & fallback,
  const std::string & doc)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = doc;
  d.read_only = true;
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback, d);
}

}  // namespace

autolivery_core::VehicleProfile declareVehicleProfile(rclcpp::Node & node)
{
  using autolivery_core::parseDropoffMethod;
  using autolivery_core::parseVehicleClass;
  autolivery_core::VehicleProfile p;
  p.name = declare<std::string>(node, "vehicle.name", p.name, "Profile name, for logs");
  p.vehicle_class = parseVehicleClass(
    declare<std::string>(node, "vehicle.class", "multicopter", "multicopter | fixed_wing | vtol"));
  p.dropoff_method = parseDropoffMethod(
    declare<std::string>(
      node, "vehicle.dropoff_method", "hover",
      "hover | winch | land | air_drop"));
  p.cruise_speed = declare(node, "vehicle.cruise_speed", p.cruise_speed, "m/s");
  p.max_speed = declare(node, "vehicle.max_speed", p.max_speed, "m/s");
  p.min_speed = declare(node, "vehicle.min_speed", p.min_speed, "m/s, fixed-wing stall margin");
  p.max_climb_rate = declare(node, "vehicle.max_climb_rate", p.max_climb_rate, "m/s");
  p.max_descent_rate = declare(node, "vehicle.max_descent_rate", p.max_descent_rate, "m/s");
  p.max_acceleration = declare(node, "vehicle.max_acceleration", p.max_acceleration, "m/s^2");
  p.cruise_altitude = declare(node, "vehicle.cruise_altitude", p.cruise_altitude, "m above depot");
  p.takeoff_altitude =
    declare(node, "vehicle.takeoff_altitude", p.takeoff_altitude, "m, fixed-wing AUTO.TAKEOFF");
  p.body_radius = declare(node, "vehicle.body_radius", p.body_radius, "m, airframe + parcel");
  p.planning_clearance =
    declare(node, "vehicle.planning_clearance", p.planning_clearance, "m kept from obstacles");
  p.acceptance_radius = declare(node, "vehicle.acceptance_radius", p.acceptance_radius, "m");
  p.turn_radius = declare(node, "vehicle.turn_radius", p.turn_radius, "m, fixed-wing");
  p.release_altitude =
    declare(node, "vehicle.release_altitude", p.release_altitude, "m above dropoff surface");
  p.max_payload_kg = declare(node, "vehicle.max_payload_kg", p.max_payload_kg, "kg");
  p.max_range = declare(node, "vehicle.max_range", p.max_range, "m of flight with reserve");
  p.max_wind = declare(node, "vehicle.max_wind", p.max_wind, "m/s, mean wind it may fly in");
  p.max_gust = declare(node, "vehicle.max_gust", p.max_gust, "m/s, gusts it may fly in");
  p.max_precipitation =
    declare(node, "vehicle.max_precipitation", p.max_precipitation, "mm/h it may fly in");
  p.min_temperature = declare(node, "vehicle.min_temperature", p.min_temperature, "deg C");
  p.max_temperature = declare(node, "vehicle.max_temperature", p.max_temperature, "deg C");
  p.px4_airframe = static_cast<int>(
    declare<int64_t>(node, "vehicle.px4_airframe", p.px4_airframe, "SYS_AUTOSTART"));
  p.sim_model = declare<std::string>(node, "vehicle.sim_model", "", "Gazebo model, if simulated");
  return p;
}

autolivery_core::MissionConfig declareMissionConfig(rclcpp::Node & node)
{
  autolivery_core::MissionConfig c;
  c.offboard_warmup = declare(node, "mission.offboard_warmup", c.offboard_warmup, "s");
  c.position_tolerance = declare(node, "mission.position_tolerance", c.position_tolerance, "m");
  c.altitude_tolerance = declare(node, "mission.altitude_tolerance", c.altitude_tolerance, "m");
  c.settle_speed = declare(node, "mission.settle_speed", c.settle_speed, "m/s");
  c.settle_time = declare(node, "mission.settle_time", c.settle_time, "s");
  c.lookahead = declare(node, "mission.lookahead", c.lookahead, "m");
  c.payload_feedback = declare(
    node, "mission.payload_feedback", c.payload_feedback,
    "Trust /autolivery/payload/state for release confirmation");
  c.release_retry_period =
    declare(node, "mission.release_retry_period", c.release_retry_period, "s");
  c.release_timeout = declare(node, "mission.release_timeout", c.release_timeout, "s");
  c.battery_return = declare(node, "mission.battery_return", c.battery_return, "fraction");
  c.battery_land = declare(node, "mission.battery_land", c.battery_land, "fraction");
  c.battery_min_start =
    declare(node, "mission.battery_min_start", c.battery_min_start, "fraction");
  c.vtol_transition_margin =
    declare(node, "mission.vtol_transition_margin", c.vtol_transition_margin, "m");
  c.air_drop_approach = declare(node, "mission.air_drop_approach", c.air_drop_approach, "m");
  c.air_drop_overrun = declare(node, "mission.air_drop_overrun", c.air_drop_overrun, "m");
  c.air_drop_latency = declare(node, "mission.air_drop_latency", c.air_drop_latency, "s");
  c.air_drop_max_cross_track =
    declare(node, "mission.air_drop_max_cross_track", c.air_drop_max_cross_track, "m");
  c.air_drop_attempts = static_cast<int>(
    declare<int64_t>(node, "mission.air_drop_attempts", c.air_drop_attempts, "passes"));
  c.telemetry_timeout = declare(
    node, "mission.telemetry_timeout", c.telemetry_timeout,
    "s without link or position in flight before leaving the vehicle to the autopilot");
  c.require_weather_report = declare(
    node, "mission.require_weather_report", c.require_weather_report,
    "Refuse deliveries without a current report on /autolivery/weather");
  c.weather_abort_after = declare(
    node, "mission.weather_abort_after", c.weather_abort_after,
    "s of weather outside the vehicle's limits in flight before the delivery is abandoned");
  c.preflight_timeout = declare(node, "mission.preflight_timeout", c.preflight_timeout, "s");
  c.takeoff_timeout = declare(node, "mission.takeoff_timeout", c.takeoff_timeout, "s");
  c.descend_timeout = declare(node, "mission.descend_timeout", c.descend_timeout, "s");
  c.land_timeout = declare(node, "mission.land_timeout", c.land_timeout, "s");
  c.leg_timeout_factor = declare(node, "mission.leg_timeout_factor", c.leg_timeout_factor, "x");
  c.leg_timeout_margin = declare(node, "mission.leg_timeout_margin", c.leg_timeout_margin, "s");
  return c;
}

autolivery_core::GripperConfig declareGripperConfig(rclcpp::Node & node)
{
  autolivery_core::GripperConfig g;
  g.type = autolivery_core::parseGripperType(
    declare<std::string>(
      node, "gripper.type", "simulated",
      "simulated | mavlink_gripper | servo | electromagnet | winch"));
  g.instance = static_cast<int>(declare<int64_t>(node, "gripper.instance", g.instance, "1-based"));
  g.actuator = static_cast<int>(
    declare<int64_t>(node, "gripper.actuator", g.actuator, "Peripheral via Actuator Set 1..6"));
  g.grab_value = static_cast<float>(
    declare<double>(node, "gripper.grab_value", g.grab_value, "actuator value [-1, 1]"));
  g.release_value = static_cast<float>(
    declare<double>(node, "gripper.release_value", g.release_value, "actuator value [-1, 1]"));
  g.winch_rate = declare(node, "gripper.winch_rate", g.winch_rate, "m/s");
  g.winch_length = declare(node, "gripper.winch_length", g.winch_length, "m, 0 = to the ground");
  g.release_time = declare(node, "gripper.release_time", g.release_time, "s");
  return g;
}

PlannerSettings declarePlannerSettings(rclcpp::Node & node)
{
  PlannerSettings s;
  s.algorithm = declare<std::string>(node, "planner.algorithm", s.algorithm, "astar | rrt_star");
  s.resolution = declare(node, "planner.resolution", s.resolution, "m, A* voxel size");
  s.max_segment = declare(node, "planner.max_segment", s.max_segment, "m, 0 = no densify");
  s.astar.climb_cost = declare(node, "planner.climb_cost", s.astar.climb_cost, ">= 1");
  s.rrt.climb_cost = s.astar.climb_cost;
  s.astar.max_expansions = static_cast<std::size_t>(
    declare<int64_t>(
      node, "planner.max_expansions", static_cast<int64_t>(s.astar.max_expansions), "A*"));
  s.rrt.max_iterations = static_cast<std::size_t>(
    declare<int64_t>(
      node, "planner.rrt_max_iterations", static_cast<int64_t>(s.rrt.max_iterations), "RRT*"));
  s.rrt.max_planning_time =
    declare(node, "planner.rrt_max_time", s.rrt.max_planning_time, "s, RRT* anytime budget");
  s.rrt.step_size = declare(node, "planner.rrt_step", s.rrt.step_size, "m");
  s.rrt.goal_bias = declare(node, "planner.rrt_goal_bias", s.rrt.goal_bias, "[0, 1]");
  s.rrt.rewire_radius = declare(node, "planner.rrt_rewire_radius", s.rrt.rewire_radius, "m");
  s.rrt.seed = static_cast<std::uint32_t>(
    declare<int64_t>(node, "planner.rrt_seed", 0, "0 = random"));
  s.rrt.min_altitude = declare(
    node, "planner.rrt_min_altitude", 10.0, "m, bottom of the RRT* sampling band");
  s.rrt.max_altitude = declare(
    node, "planner.rrt_max_altitude", 100.0, "m, top of the RRT* sampling band");
  if (s.algorithm != "astar" && s.algorithm != "rrt_star") {
    throw std::invalid_argument("planner.algorithm must be astar or rrt_star");
  }
  return s;
}

AvoidanceSettings declareAvoidanceSettings(
  rclcpp::Node & node, const autolivery_core::VehicleProfile & profile)
{
  AvoidanceSettings s;
  s.enabled = declare(node, "avoidance.enabled", s.enabled, "React to /autolivery/obstacles");
  s.obstacle_timeout = declare(node, "avoidance.obstacle_timeout", s.obstacle_timeout, "s");
  auto & p = s.params;
  p.time_horizon = declare(node, "avoidance.time_horizon", p.time_horizon, "s");
  p.safety_margin = declare(node, "avoidance.safety_margin", p.safety_margin, "m");
  p.heading_samples = static_cast<int>(
    declare<int64_t>(node, "avoidance.heading_samples", p.heading_samples, "per side"));
  p.static_clearance = declare(node, "avoidance.static_clearance", p.static_clearance, "m");
  p.static_horizon = declare(node, "avoidance.static_horizon", p.static_horizon, "s");
  p.vehicle_radius = profile.body_radius;
  p.max_horizontal_speed = profile.max_speed;
  p.max_climb_rate = profile.max_climb_rate;
  p.max_descent_rate = profile.max_descent_rate;
  p.min_speed = profile.min_speed;
  return s;
}

std::string resolvePackagePath(const std::string & value)
{
  if (value.empty() || value.front() == '/') {
    return value;
  }
  return ament_index_cpp::get_package_share_directory("autolivery_drone") + "/" + value;
}

}  // namespace autolivery_drone
