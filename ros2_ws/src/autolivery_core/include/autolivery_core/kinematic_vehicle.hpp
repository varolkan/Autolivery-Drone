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
//   * VTOL transitions take time, and back-transitions shed airspeed; with `quadchute` set a
//     front transition times out and falls back to multicopter, as PX4's quad-chute does
//   * wind: a hovering vehicle holds its ground velocity only while that takes no more
//     airspeed than it has, and is nudged by gusts; a plane flies its airspeed through the
//     moving air, crabbing to hold its course, so its ground speed is whatever is left
// It is not a flight dynamics model: there is no attitude and no drag.

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
  const Vec3 & airVelocity() const {return air_;}  // the wind on the vehicle right now
  double heading() const {return heading_;}
  void place(const Vec3 & p) {pos_ = p;}

  // Knobs.
  bool connected{true};
  bool position_ok{true};  // false: the link is up but no position comes through it
  double battery{1.0};
  double drain_per_s{0.0};
  bool gripper_stuck{false};
  bool payload_sensor{true};
  std::string mode{"AUTO.LOITER"};
  bool armed{false};
  double runway_heading{0.0};  // rad ENU, direction of a fixed-wing AUTO.TAKEOFF roll
  Vec3 wind;                   // m/s, world frame: the way the air is moving (the mean)
  double gust{0.0};            // m/s the gusts swing the wind by, either side of the mean
  // What the mission is told about the weather, as a station's report would. Not what
  // blows -- that is `wind` -- so a test can make the two disagree.
  std::optional<Weather> reported_weather;
  bool fixed_wing_rtl_lands{false};  // a mission landing pattern is loaded (PX4 RTL_TYPE 1)
  bool quadchute{false};  // front transitions time out and it is a multicopter again

  // What happened.
  std::vector<Vec3> trajectory;
  bool record_trajectory{true};
  std::optional<Vec3> release_position;
  std::optional<Vec3> release_velocity;
  int release_commands{0};
  bool saw_fixed_wing{false};
  int quadchutes{0};

private:
  bool isPlane() const;
  double ground() const;
  Vec3 windAt(double now) const;
  Vec3 reachable(const Vec3 & ground_velocity) const;
  void flyPlaneToward(const Vec3 & target, double dt);
  void approachVelocity(const Vec3 & target_vel, double dt);
  void fly(double dt);

  VehicleProfile profile_;
  Vec3 home_;
  std::shared_ptr<const WorldMap> map_;
  Vec3 pos_;
  Vec3 vel_;
  Vec3 air_;                   // the wind right now
  bool air_felt_{false};
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
