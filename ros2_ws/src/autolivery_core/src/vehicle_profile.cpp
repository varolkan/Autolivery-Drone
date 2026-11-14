#include "autolivery_core/vehicle_profile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace autolivery_core
{

namespace
{

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

}  // namespace

VehicleClass parseVehicleClass(const std::string & text)
{
  const std::string t = lower(text);
  if (t == "multicopter" || t == "multirotor") {
    return VehicleClass::Multicopter;
  }
  if (t == "fixed_wing" || t == "fixedwing" || t == "plane") {
    return VehicleClass::FixedWing;
  }
  if (t == "vtol") {
    return VehicleClass::Vtol;
  }
  throw std::invalid_argument(
          "unknown vehicle class '" + text + "' (expected multicopter, fixed_wing or vtol)");
}

DropoffMethod parseDropoffMethod(const std::string & text)
{
  const std::string t = lower(text);
  if (t == "hover") {
    return DropoffMethod::Hover;
  }
  if (t == "winch") {
    return DropoffMethod::Winch;
  }
  if (t == "land") {
    return DropoffMethod::Land;
  }
  if (t == "air_drop" || t == "airdrop") {
    return DropoffMethod::AirDrop;
  }
  throw std::invalid_argument(
          "unknown dropoff method '" + text + "' (expected hover, winch, land or air_drop)");
}

std::string toString(VehicleClass c)
{
  switch (c) {
    case VehicleClass::Multicopter: return "multicopter";
    case VehicleClass::FixedWing: return "fixed_wing";
    case VehicleClass::Vtol: return "vtol";
  }
  return "unknown";
}

std::string toString(DropoffMethod m)
{
  switch (m) {
    case DropoffMethod::Hover: return "hover";
    case DropoffMethod::Winch: return "winch";
    case DropoffMethod::Land: return "land";
    case DropoffMethod::AirDrop: return "air_drop";
  }
  return "unknown";
}

double VehicleProfile::groundSpeed(const Vec3 & course, const Vec3 & wind) const
{
  // Along its course a vehicle gains or loses the wind's along-track part, and holding the
  // course against the cross-track part costs airspeed as well. A multirotor is told a ground
  // speed and makes it for as long as that takes no more airspeed than it has; a wing flies
  // its airspeed and gets what is left.
  constexpr double kSlowest = 1.0;
  const double along = wind.dot(course);
  const double cross = (wind - course * along).horizontalNorm();
  const double airspeed = legAirspeed();
  const double made_good =
    cross < airspeed ? std::sqrt(airspeed * airspeed - cross * cross) + along : 0.0;
  const bool multicopter = vehicle_class == VehicleClass::Multicopter;
  return std::max(kSlowest, multicopter ? std::min(cruise_speed, made_good) : made_good);
}

std::vector<std::string> validate(const VehicleProfile & p)
{
  std::vector<std::string> problems;
  auto positive = [&](double v, const char * name) {
      if (!(v > 0.0) || !std::isfinite(v)) {
        problems.push_back(std::string(name) + " must be positive");
      }
    };
  positive(p.cruise_speed, "cruise_speed");
  positive(p.max_speed, "max_speed");
  positive(p.max_climb_rate, "max_climb_rate");
  positive(p.max_descent_rate, "max_descent_rate");
  positive(p.max_acceleration, "max_acceleration");
  positive(p.cruise_altitude, "cruise_altitude");
  positive(p.takeoff_altitude, "takeoff_altitude");
  positive(p.body_radius, "body_radius");
  positive(p.acceptance_radius, "acceptance_radius");
  positive(p.release_altitude, "release_altitude");
  positive(p.max_range, "max_range");

  if (p.cruise_speed > p.max_speed) {
    problems.push_back("cruise_speed exceeds max_speed");
  }
  if (p.min_speed < 0.0 || p.min_speed >= p.cruise_speed) {
    problems.push_back("min_speed must be in [0, cruise_speed)");
  }
  if (p.planning_clearance < p.body_radius) {
    // The planner clearance is measured from the vehicle's centre; anything smaller than the
    // body radius plans paths that clip buildings with the propellers.
    problems.push_back("planning_clearance must be at least body_radius");
  }
  if (p.takeoff_altitude > p.cruise_altitude) {
    problems.push_back("takeoff_altitude must not exceed cruise_altitude");
  }

  positive(p.max_wind, "max_wind");
  if (p.max_gust < p.max_wind || !std::isfinite(p.max_gust)) {
    problems.push_back("max_gust must be at least max_wind");
  }
  if (p.max_wind >= p.legAirspeed()) {
    // In a wind as fast as the vehicle it stands still over the ground, or goes backwards.
    problems.push_back(
      std::string("max_wind must be below ") +
      (p.vehicle_class == VehicleClass::Multicopter ? "max_speed" : "cruise_speed") +
      ": the vehicle cannot make headway against a wind it cannot outfly");
  }
  if (!(p.max_precipitation >= 0.0) || !std::isfinite(p.max_precipitation)) {
    problems.push_back("max_precipitation must not be negative");
  }
  if (!(p.min_temperature < p.max_temperature)) {
    problems.push_back("min_temperature must be below max_temperature");
  }

  switch (p.vehicle_class) {
    case VehicleClass::FixedWing:
      if (p.dropoff_method != DropoffMethod::AirDrop) {
        problems.push_back(
          "a fixed-wing vehicle cannot hover, so its dropoff_method must be air_drop, not " +
          toString(p.dropoff_method));
      }
      if (!(p.min_speed > 0.0)) {
        problems.push_back("fixed-wing profiles need min_speed above stall");
      }
      if (!(p.turn_radius > 0.0)) {
        problems.push_back("fixed-wing profiles need a positive turn_radius");
      }
      if (p.acceptance_radius < p.turn_radius) {
        // A plane cannot hit a waypoint tighter than it can turn; it orbits it instead.
        problems.push_back("fixed-wing acceptance_radius must be at least turn_radius");
      }
      break;
    case VehicleClass::Vtol:
      if (!(p.turn_radius > 0.0)) {
        problems.push_back("VTOL profiles need a positive turn_radius for the fixed-wing legs");
      }
      break;
    case VehicleClass::Multicopter:
      if (p.min_speed != 0.0) {
        problems.push_back("multicopter profiles must have min_speed 0");
      }
      break;
  }
  return problems;
}

}  // namespace autolivery_core
