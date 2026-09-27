#include "autolivery_core/gripper.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autolivery_core
{

namespace
{

// MAV_CMD_DO_WINCH param2 (WINCH_ACTIONS).
constexpr float kWinchRelativeLengthControl = 1.0f;
constexpr float kWinchLock = 3.0f;
constexpr float kWinchDeliver = 4.0f;
constexpr float kWinchRetract = 6.0f;

// MAV_CMD_DO_GRIPPER param2 (GRIPPER_ACTIONS).
constexpr float kGripperRelease = 0.0f;
constexpr float kGripperGrab = 1.0f;

MavCommand setActuator(int actuator, float value, const std::string & description)
{
  if (actuator < 1 || actuator > 6) {
    throw std::invalid_argument("gripper actuator must be in 1..6 (Peripheral via Actuator Set)");
  }
  if (!(value >= -1.0f && value <= 1.0f)) {
    throw std::invalid_argument("gripper actuator values must be in [-1, 1]");
  }
  MavCommand cmd;
  cmd.command = kMavCmdDoSetActuator;
  // NaN means "leave this output alone". Sending 0 instead would drive the other five
  // peripheral outputs to their centre position -- including, on a real airframe, whatever
  // else is wired to them.
  cmd.params.fill(std::numeric_limits<float>::quiet_NaN());
  cmd.params[actuator - 1] = value;
  cmd.params[6] = 0.0f;  // index: outputs 1..6
  cmd.description = description;
  return cmd;
}

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

}  // namespace

GripperType parseGripperType(const std::string & text)
{
  const std::string t = lower(text);
  if (t == "simulated" || t == "gazebo" || t == "sim") {
    return GripperType::Simulated;
  }
  if (t == "mavlink_gripper" || t == "mavlink") {
    return GripperType::MavlinkGripper;
  }
  if (t == "servo") {
    return GripperType::Servo;
  }
  if (t == "electromagnet" || t == "magnet") {
    return GripperType::Electromagnet;
  }
  if (t == "winch") {
    return GripperType::Winch;
  }
  throw std::invalid_argument(
          "unknown gripper type '" + text +
          "' (expected simulated, mavlink_gripper, servo, electromagnet or winch)");
}

std::string toString(GripperType type)
{
  switch (type) {
    case GripperType::Simulated: return "simulated";
    case GripperType::MavlinkGripper: return "mavlink_gripper";
    case GripperType::Servo: return "servo";
    case GripperType::Electromagnet: return "electromagnet";
    case GripperType::Winch: return "winch";
  }
  return "unknown";
}

std::vector<MavCommand> gripperCommands(const GripperConfig & config, GripperAction action)
{
  const bool grab = action == GripperAction::Grab;
  switch (config.type) {
    case GripperType::Simulated:
      return {};

    case GripperType::MavlinkGripper: {
        if (config.instance < 1) {
          throw std::invalid_argument("gripper instance must be >= 1");
        }
        MavCommand cmd;
        cmd.command = kMavCmdDoGripper;
        cmd.params[0] = static_cast<float>(config.instance);
        cmd.params[1] = grab ? kGripperGrab : kGripperRelease;
        cmd.description = grab ? "gripper grab" : "gripper release";
        return {cmd};
      }

    case GripperType::Servo:
      return {setActuator(
          config.actuator, grab ? config.grab_value : config.release_value,
          grab ? "servo close" : "servo open")};

    case GripperType::Electromagnet:
      return {setActuator(
          config.actuator, grab ? config.grab_value : config.release_value,
          grab ? "electromagnet on" : "electromagnet off")};

    case GripperType::Winch: {
        if (config.instance < 1) {
          throw std::invalid_argument("winch instance must be >= 1");
        }
        MavCommand cmd;
        cmd.command = kMavCmdDoWinch;
        cmd.params[0] = static_cast<float>(config.instance);
        if (grab) {
          // Reel the (empty) line back in, then hold it so it cannot pay out in flight.
          cmd.params[1] = kWinchRetract;
          cmd.params[3] = static_cast<float>(config.winch_rate);
          cmd.description = "winch retract";
          MavCommand lock;
          lock.command = kMavCmdDoWinch;
          lock.params[0] = static_cast<float>(config.instance);
          lock.params[1] = kWinchLock;
          lock.description = "winch lock";
          return {cmd, lock};
        }
        if (config.winch_length > 0.0) {
          cmd.params[1] = kWinchRelativeLengthControl;
          cmd.params[2] = static_cast<float>(config.winch_length);
          cmd.params[3] = static_cast<float>(config.winch_rate);
          cmd.description = "winch lower";
        } else {
          cmd.params[1] = kWinchDeliver;
          cmd.params[3] = static_cast<float>(config.winch_rate);
          cmd.description = "winch deliver";
        }
        return {cmd};
      }
  }
  return {};
}

double releaseDuration(const GripperConfig & config, double release_altitude)
{
  if (config.type == GripperType::Winch && config.winch_rate > 0.0) {
    const double line = config.winch_length > 0.0 ? config.winch_length : release_altitude;
    return config.release_time + line / config.winch_rate;
  }
  return config.release_time;
}

}  // namespace autolivery_core
