// Payload release hardware, reduced to the MAVLink commands that drive it.
//
// PX4 owns the actuator outputs, so every real gripper is driven through the flight
// controller: MAV_CMD_DO_GRIPPER for PX4's payload deliverer, MAV_CMD_DO_SET_ACTUATOR for a
// servo claw or electromagnet mapped to a "Peripheral via Actuator Set" output, and
// MAV_CMD_DO_WINCH for a tether winch. Keeping this as data (which command, which params)
// rather than code lets the mapping be unit tested without a flight controller, and lets the
// ROS layer send whatever comes out over /mavros/cmd/command without knowing gripper types.
//
// The simulated gripper has no MAVLink side: the ROS layer detaches the parcel in Gazebo.

#ifndef AUTOLIVERY_CORE__GRIPPER_HPP_
#define AUTOLIVERY_CORE__GRIPPER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autolivery_core
{

// MAVLink command ids used here (common.xml).
constexpr std::uint16_t kMavCmdDoGripper = 211;
constexpr std::uint16_t kMavCmdDoSetActuator = 187;
constexpr std::uint16_t kMavCmdDoWinch = 42600;
constexpr std::uint16_t kMavCmdDoVtolTransition = 3000;

// MAV_VTOL_STATE values for MAV_CMD_DO_VTOL_TRANSITION param1.
constexpr float kMavVtolStateMc = 3.0f;
constexpr float kMavVtolStateFw = 4.0f;

enum class GripperType
{
  Simulated,       // Gazebo DetachableJoint, driven by the ROS layer
  MavlinkGripper,  // MAV_CMD_DO_GRIPPER (PX4 payload_deliverer)
  Servo,           // MAV_CMD_DO_SET_ACTUATOR, claw or hook on a servo
  Electromagnet,   // MAV_CMD_DO_SET_ACTUATOR, energised to hold
  Winch,           // MAV_CMD_DO_WINCH, lowers the parcel on a tether then releases it
};

enum class GripperAction
{
  Grab,
  Release,
};

struct GripperConfig
{
  GripperType type{GripperType::Simulated};
  int instance{1};             // gripper / winch instance (1-based, as MAVLink expects)
  int actuator{1};             // "Peripheral via Actuator Set" output number, 1..6
  float grab_value{1.0f};      // actuator value in [-1, 1] that holds the parcel
  float release_value{-1.0f};  // actuator value in [-1, 1] that lets it go
  double winch_rate{0.5};      // m/s the tether pays out at
  double winch_length{0.0};    // tether to pay out; 0 = release altitude
  // How long after the release command the parcel can be assumed gone when there is no
  // payload sensor. Electromagnets need a moment to demagnetise; a winch needs the whole
  // lowering time, which the ROS layer adds from the release altitude.
  double release_time{1.5};
};

struct MavCommand
{
  std::uint16_t command{0};
  std::array<float, 7> params{};
  std::string description;
};

GripperType parseGripperType(const std::string & text);
std::string toString(GripperType type);

// Commands that perform `action` on the configured gripper, in order. Empty for the
// simulated gripper. Throws std::invalid_argument on a config that would send nonsense (an
// actuator index outside 1..6, a value outside [-1, 1]).
std::vector<MavCommand> gripperCommands(const GripperConfig & config, GripperAction action);

// Seconds to wait for a release to complete when there is no payload sensor.
double releaseDuration(const GripperConfig & config, double release_altitude);

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__GRIPPER_HPP_
