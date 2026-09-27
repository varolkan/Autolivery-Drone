// A kinematic stand-in for PX4, for dry runs and tests.
//
// It reproduces the parts of PX4's behaviour the mission depends on, so the mission is
// exercised through the same handshakes as a real flight:
//   * OFFBOARD is only accepted after setpoints have been streaming for a second
//   * arming is refused outside OFFBOARD and AUTO.TAKEOFF
//   * AUTO.LAND descends and auto-disarms a second after touchdown, then reverts to the last
//     commanded mode that allows arming (usually OFFBOARD), like PX4's commander
//   * AUTO.TAKEOFF drops into AUTO.LOITER when the takeoff altitude is reached
//   * fixed-wings fly at constant airspeed and cannot turn tighter than their radius; their
//     AUTO.LAND is a circular landing, and their AUTO.RTL loiters over home without landing
//   * VTOL transitions take time, and back-transitions shed airspeed
// It is not a flight dynamics model: there is no attitude, no wind, no drag.

#ifndef AUTOLIVERY_CORE__KINEMATIC_VEHICLE_HPP_
#define AUTOLIVERY_CORE__KINEMATIC_VEHICLE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "autolivery_core/mission.hpp"
#include "autolivery_core/vehicle_profile.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

class KinematicVehicle
{
public:
  KinematicVehicle(VehicleProfile profile, Vec3 home, std::shared_ptr<const WorldMap> map);

  // Inputs, as MAVROS would deliver them.
  void setSetpoint(const Setpoint & setpoint, double now);
  bool requestMode(const std::string & mode, double now);  // true if accepted
  bool requestArm(bool arm);                                // true if accepted
  void requestVtol(VtolRequest request, double now);
  void releasePayload();
  void attachPayload();

  // Advances the simulation by dt seconds ending at `now`.
  void step(double now, double dt);

  // Test convenience: apply a mission command and step, like the ROS node would.
  void step(const MissionCommand & cmd, double now, double dt);

  VehicleSnapshot snapshot() const;
  bool onGround() const;
  PayloadState payload() const {return payload_;}
  VtolState vtol() const {return vtol_;}
  const Vec3 & position() const {return pos_;}
  const Vec3 & velocity() const {return vel_;}
  double heading() const {return heading_;}
  void place(const Vec3 & p) {pos_ = p;}

  // Knobs.
  bool connected{true};
  double battery{1.0};
  double drain_per_s{0.0};
  bool gripper_stuck{false};
  bool payload_sensor{true};
  std::string mode{"AUTO.LOITER"};
  bool armed{false};
  double runway_heading{0.0};  // rad ENU, direction of a fixed-wing AUTO.TAKEOFF roll
  bool fixed_wing_rtl_lands{false};  // a mission landing pattern is loaded (PX4 RTL_TYPE 1)

  // What happened.
  std::vector<Vec3> trajectory;
  bool record_trajectory{true};
  std::optional<Vec3> release_position;
  std::optional<Vec3> release_velocity;
  int release_commands{0};
  bool saw_fixed_wing{false};

private:
  bool isPlane() const;
  double ground() const;
  void flyPlaneToward(const Vec3 & target, double dt);
  void approachVelocity(const Vec3 & target_vel, double dt);
  void fly(double dt);

  VehicleProfile profile_;
  Vec3 home_;
  std::shared_ptr<const WorldMap> map_;
  Vec3 pos_;
  Vec3 vel_;
  double heading_{0.0};
  Setpoint sp_;
  double stream_start_{-1.0};
  double last_setpoint_{-1.0};
  double landed_since_{-1.0};
  double transition_start_{0.0};
  VtolState vtol_{VtolState::Undefined};
  std::string after_disarm_mode_{"AUTO.LOITER"};
  PayloadState payload_{PayloadState::Attached};
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__KINEMATIC_VEHICLE_HPP_
