// Conversions between ROS/MAVROS messages and autolivery_core types.
//
// Frames: the core works in the world frame (city map, ENU, origin at the main depot). MAVROS
// reports and accepts the autopilot's local frame, also ENU (MAVROS does the NED rotation),
// with its origin wherever the EKF initialised -- the depot pad the vehicle powered up on.
// So world = local + depot, and nothing more is needed as long as the vehicle is switched on
// at its depot. The mission refuses to start if the vehicle is not where the depot is.

#ifndef AUTOLIVERY_DRONE__CONVERSIONS_HPP_
#define AUTOLIVERY_DRONE__CONVERSIONS_HPP_

#include <cmath>

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_interfaces/msg/weather.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"

namespace autolivery_drone
{

inline autolivery_core::Vec3 toVec3(const geometry_msgs::msg::Point & p) {return {p.x, p.y, p.z};}
inline autolivery_core::Vec3 toVec3(const geometry_msgs::msg::Vector3 & v) {return {v.x, v.y, v.z};}

inline geometry_msgs::msg::Point toPoint(const autolivery_core::Vec3 & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

inline geometry_msgs::msg::Vector3 toVector3(const autolivery_core::Vec3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline autolivery_core::LandedState landedFromMavros(uint8_t s)
{
  using E = mavros_msgs::msg::ExtendedState;
  switch (s) {
    case E::LANDED_STATE_ON_GROUND: return autolivery_core::LandedState::OnGround;
    case E::LANDED_STATE_IN_AIR: return autolivery_core::LandedState::InAir;
    case E::LANDED_STATE_TAKEOFF: return autolivery_core::LandedState::Takeoff;
    case E::LANDED_STATE_LANDING: return autolivery_core::LandedState::Landing;
    default: return autolivery_core::LandedState::Unknown;
  }
}

inline uint8_t landedToMavros(autolivery_core::LandedState s)
{
  using E = mavros_msgs::msg::ExtendedState;
  switch (s) {
    case autolivery_core::LandedState::OnGround: return E::LANDED_STATE_ON_GROUND;
    case autolivery_core::LandedState::InAir: return E::LANDED_STATE_IN_AIR;
    case autolivery_core::LandedState::Takeoff: return E::LANDED_STATE_TAKEOFF;
    case autolivery_core::LandedState::Landing: return E::LANDED_STATE_LANDING;
    default: return E::LANDED_STATE_UNDEFINED;
  }
}

inline autolivery_core::VtolState vtolFromMavros(uint8_t s)
{
  using E = mavros_msgs::msg::ExtendedState;
  switch (s) {
    case E::VTOL_STATE_MC: return autolivery_core::VtolState::Mc;
    case E::VTOL_STATE_FW: return autolivery_core::VtolState::Fw;
    case E::VTOL_STATE_TRANSITION_TO_FW: return autolivery_core::VtolState::TransitionToFw;
    case E::VTOL_STATE_TRANSITION_TO_MC: return autolivery_core::VtolState::TransitionToMc;
    default: return autolivery_core::VtolState::Undefined;
  }
}

inline uint8_t vtolToMavros(autolivery_core::VtolState s)
{
  using E = mavros_msgs::msg::ExtendedState;
  switch (s) {
    case autolivery_core::VtolState::Mc: return E::VTOL_STATE_MC;
    case autolivery_core::VtolState::Fw: return E::VTOL_STATE_FW;
    case autolivery_core::VtolState::TransitionToFw: return E::VTOL_STATE_TRANSITION_TO_FW;
    case autolivery_core::VtolState::TransitionToMc: return E::VTOL_STATE_TRANSITION_TO_MC;
    default: return E::VTOL_STATE_UNDEFINED;
  }
}

// Mission setpoint (world frame) -> MAVROS setpoint_raw/local (autopilot local frame).
//
// FRAME_LOCAL_NED is what MAVROS expects on this topic even though the values are ENU: MAVROS
// rotates ENU->NED itself and only uses the frame id to pick the MAVLink frame.
inline mavros_msgs::msg::PositionTarget toPositionTarget(
  const autolivery_core::Setpoint & sp, const autolivery_core::Vec3 & local_origin_in_world)
{
  using PT = mavros_msgs::msg::PositionTarget;
  PT out;
  out.coordinate_frame = PT::FRAME_LOCAL_NED;
  uint16_t mask = PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ | PT::IGNORE_YAW_RATE;
  if (sp.kind == autolivery_core::Setpoint::Kind::Velocity) {
    mask |= PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ;
    out.velocity = toVector3(sp.velocity);
  } else {
    mask |= PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ;
    out.position = toPoint(sp.position - local_origin_in_world);
  }
  if (std::isnan(sp.yaw)) {
    mask |= PT::IGNORE_YAW;
  } else {
    out.yaw = static_cast<float>(sp.yaw);
  }
  out.type_mask = mask;
  return out;
}

// The inverse, used by the fake FCU: what a PositionTarget asks for, in the world frame.
inline autolivery_core::Setpoint fromPositionTarget(
  const mavros_msgs::msg::PositionTarget & pt, const autolivery_core::Vec3 & local_origin_in_world)
{
  using PT = mavros_msgs::msg::PositionTarget;
  autolivery_core::Setpoint sp;
  const bool has_position = (pt.type_mask & (PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ)) == 0;
  const bool has_velocity = (pt.type_mask & (PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ)) == 0;
  if (has_position) {
    sp.kind = autolivery_core::Setpoint::Kind::Position;
    sp.position = toVec3(pt.position) + local_origin_in_world;
  } else if (has_velocity) {
    sp.kind = autolivery_core::Setpoint::Kind::Velocity;
    sp.velocity = toVec3(pt.velocity);
  }
  if ((pt.type_mask & PT::IGNORE_YAW) == 0) {
    sp.yaw = pt.yaw;
  }
  return sp;
}

// A wind the way meteorology gives it -- a speed and the compass direction it blows FROM --
// as the velocity of the air in the world frame (x east, y north).
inline autolivery_core::Vec3 windFromMeteorological(double speed, double from_deg)
{
  const double from = from_deg * autolivery_core::kPi / 180.0;
  return {-speed * std::sin(from), -speed * std::cos(from), 0.0};
}

// The direction a wind blows from, degrees clockwise from north in [0, 360).
inline double windDirection(const autolivery_core::Vec3 & wind)
{
  const double from = std::atan2(-wind.x, -wind.y) * 180.0 / autolivery_core::kPi;
  return from < 0.0 ? from + 360.0 : from;
}

// A weather report as the mission wants it. Fields the report left out (NaN) stay unknown.
inline autolivery_core::Weather weatherFromMsg(const autolivery_interfaces::msg::Weather & m)
{
  autolivery_core::Weather w;
  if (std::isfinite(m.wind_speed) && m.wind_speed >= 0.0f) {
    // "Light and variable" comes without a direction. The speed still counts against the
    // limit; which way it blows only matters for timing, and at those speeds hardly at all.
    w.wind = windFromMeteorological(
      m.wind_speed, std::isfinite(m.wind_direction) ? m.wind_direction : 0.0);
  }
  if (std::isfinite(m.wind_gust) && m.wind_gust >= 0.0f) {
    w.gust = m.wind_gust;
  }
  if (std::isfinite(m.precipitation) && m.precipitation >= 0.0f) {
    w.precipitation = m.precipitation;
  }
  if (std::isfinite(m.temperature)) {
    w.temperature = m.temperature;
  }
  return w;
}

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__CONVERSIONS_HPP_
