// What the mission needs to know about the airframe it is flying.
//
// Every PX4 multirotor (quad, hexa, octo, coaxial, Y6, tri) flies the same way from the
// mission's point of view: it can hover, take off and land vertically, and follow velocity
// setpoints. Fixed-wing aircraft cannot hover, need position setpoints and a turn radius,
// and can only deliver by air drop. VTOLs are multirotors at both ends of the flight and
// fixed-wing in between. So the mission branches on VehicleClass, and everything else --
// speeds, radii, how the parcel comes off -- is data in a profile.

#ifndef AUTOLIVERY_CORE__VEHICLE_PROFILE_HPP_
#define AUTOLIVERY_CORE__VEHICLE_PROFILE_HPP_

#include <string>
#include <vector>

#include "autolivery_core/geometry.hpp"

namespace autolivery_core
{

enum class VehicleClass
{
  Multicopter,
  FixedWing,
  Vtol,
};

enum class DropoffMethod
{
  Hover,    // descend to release_altitude over the dropoff and open the gripper
  Winch,    // hover at release_altitude and lower the parcel on a tether
  Land,     // land at the dropoff, release on the ground, take off again
  AirDrop,  // release in forward flight, leading the target by the fall time
};

struct VehicleProfile
{
  std::string name{"quadcopter"};
  VehicleClass vehicle_class{VehicleClass::Multicopter};
  DropoffMethod dropoff_method{DropoffMethod::Hover};

  // Envelope. Keep these at or below the matching PX4 parameters (MPC_XY_VEL_MAX,
  // MPC_Z_VEL_MAX_UP/DN, FW_AIRSPD_MIN/TRIM/MAX); the autopilot clamps anyway, but a mission
  // that plans with speeds the vehicle will not fly gets its timeouts and air-drop leads wrong.
  double cruise_speed{8.0};        // m/s
  double max_speed{12.0};          // m/s
  double min_speed{0.0};           // m/s, fixed-wing stall margin (0 for hovering vehicles)
  double max_climb_rate{3.0};      // m/s
  double max_descent_rate{1.5};    // m/s
  double max_acceleration{2.0};    // m/s^2, used to start slowing down before waypoints

  // Altitudes, metres above the depot.
  double cruise_altitude{40.0};
  // Fixed-wing AUTO.TAKEOFF target; multirotors climb straight to cruise altitude.
  double takeoff_altitude{10.0};

  // Geometry.
  double body_radius{0.6};         // radius of a sphere enclosing the airframe and parcel
  double planning_clearance{4.0};  // kept from every obstacle by the global planner
  double acceptance_radius{2.0};   // waypoint reached within this distance
  double turn_radius{0.0};         // fixed-wing loiter/turn radius (0 for hovering vehicles)

  // Delivery.
  double release_altitude{3.0};    // metres above the dropoff surface when releasing
  double max_payload_kg{1.5};
  double max_range{8000.0};        // metres of flight the battery supports with reserve

  // Weather the airframe may fly in, from its manual. Outside these a delivery is refused,
  // and one that is under way is abandoned and flown home. The wind limits are for the wind
  // at flight height; a 10 m mast in a city reads well below it.
  double max_wind{8.0};            // m/s, mean wind
  double max_gust{12.0};           // m/s
  double max_precipitation{0.1};   // mm/h; the default allows no more than a trace
  double min_temperature{-10.0};   // deg C
  double max_temperature{40.0};    // deg C

  // PX4 / simulation.
  int px4_airframe{4001};          // SYS_AUTOSTART the profile was tuned against
  std::string sim_model;           // Gazebo model name, empty if the airframe is not simulated

  bool canHover() const {return vehicle_class != VehicleClass::FixedWing;}
  // Airspeed there is to work with on a cruise leg: a multirotor can lean into the wind up to
  // its top speed, anything with a wing flies its cruise airspeed.
  double legAirspeed() const
  {
    return vehicle_class == VehicleClass::Multicopter ? max_speed : cruise_speed;
  }
  // Ground speed to expect on a cruise leg along `course` (unit, horizontal) in a mean wind
  // `wind` (the way the air is moving). Never less than 1 m/s, so that a leg always has an end.
  double groundSpeed(const Vec3 & course, const Vec3 & wind) const;
};

VehicleClass parseVehicleClass(const std::string & text);
DropoffMethod parseDropoffMethod(const std::string & text);
std::string toString(VehicleClass c);
std::string toString(DropoffMethod m);

// Human-readable problems with a profile. An empty result means it is safe to fly.
std::vector<std::string> validate(const VehicleProfile & profile);

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__VEHICLE_PROFILE_HPP_
