#include "autolivery_core/kinematic_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autolivery_core
{

namespace
{

// A setpoint stream with a gap longer than this is a new stream (PX4's offboard timeout).
constexpr double kStreamGap = 0.5;

}  // namespace

KinematicVehicle::KinematicVehicle(
  VehicleProfile profile, Vec3 home, std::shared_ptr<const WorldMap> map)
: profile_(std::move(profile)), home_(home), map_(std::move(map)), pos_(home)
{
  if (profile_.vehicle_class == VehicleClass::Vtol) {
    vtol_ = VtolState::Mc;
  }
}

void KinematicVehicle::setSetpoint(const Setpoint & setpoint, double now)
{
  if (setpoint.kind == Setpoint::Kind::None) {
    return;
  }
  if (last_setpoint_ < 0.0 || now - last_setpoint_ > kStreamGap) {
    stream_start_ = now;
  }
  last_setpoint_ = now;
  sp_ = setpoint;
}

bool KinematicVehicle::requestMode(const std::string & requested, double now)
{
  if (requested == mode) {
    return true;
  }
  if (requested == px4_mode::kOffboard) {
    const bool streaming = last_setpoint_ >= 0.0 && now - last_setpoint_ <= kStreamGap &&
      now - stream_start_ >= 1.0;
    if (!streaming) {
      return false;
    }
  }
  mode = requested;
  // PX4 (UserModeIntention) remembers the last commanded mode that allows arming and
  // returns to it on disarm; Land and RTL prevent arming and are never remembered.
  if (requested != px4_mode::kLand && requested != px4_mode::kRtl) {
    after_disarm_mode_ = requested;
  }
  return true;
}

bool KinematicVehicle::requestArm(bool arm)
{
  if (arm && !armed) {
    if (mode == px4_mode::kOffboard || mode == px4_mode::kTakeoff) {
      armed = true;
    }
    return armed;
  }
  if (!arm && armed) {
    if (onGround()) {
      armed = false;
    }
    return !armed;
  }
  return true;
}

void KinematicVehicle::requestVtol(VtolRequest request, double now)
{
  if (profile_.vehicle_class != VehicleClass::Vtol) {
    return;
  }
  const bool airborne = pos_.z - ground() > 10.0;
  const bool forward_flight = vtol_ == VtolState::Fw || vtol_ == VtolState::TransitionToFw;
  if (request == VtolRequest::Fw && vtol_ == VtolState::Mc && airborne) {
    vtol_ = VtolState::TransitionToFw;
    transition_start_ = now;
  } else if (request == VtolRequest::Mc && forward_flight) {
    vtol_ = VtolState::TransitionToMc;
    transition_start_ = now;
  }
}

void KinematicVehicle::releasePayload()
{
  ++release_commands;
  if (!gripper_stuck && payload_ == PayloadState::Attached) {
    payload_ = PayloadState::Released;
    release_position = pos_;
    release_velocity = vel_;
  }
}

void KinematicVehicle::attachPayload()
{
  payload_ = PayloadState::Attached;
  release_position.reset();
  release_velocity.reset();
}

void KinematicVehicle::step(const MissionCommand & cmd, double now, double dt)
{
  setSetpoint(cmd.setpoint, now);
  if (!cmd.mode.empty()) {
    requestMode(cmd.mode, now);
  }
  if (cmd.armed.has_value()) {
    requestArm(*cmd.armed);
  }
  if (cmd.vtol != VtolRequest::None) {
    requestVtol(cmd.vtol, now);
  }
  if (cmd.release) {
    releasePayload();
  }
  step(now, dt);
}

void KinematicVehicle::step(double now, double dt)
{
  if (vtol_ == VtolState::TransitionToFw && now - transition_start_ > 3.0) {
    if (quadchute) {
      vtol_ = VtolState::Mc;
      ++quadchutes;
    } else {
      vtol_ = VtolState::Fw;
    }
  }
  if (vtol_ == VtolState::TransitionToMc && now - transition_start_ > 3.0) {
    vtol_ = VtolState::Mc;
    vel_ = vel_ * 0.3;  // back-transition sheds most of the airspeed
  }
  // PX4 falls back to Hold when the offboard stream stops (COM_OBL_RC_ACT default).
  if (mode == px4_mode::kOffboard && armed && now - last_setpoint_ > kStreamGap) {
    mode = px4_mode::kLoiter;
  }

  const Vec3 air = windAt(now);
  if (!air_felt_) {
    // The wind was there before the first step; only a change in it is a gust.
    air_ = air;
    air_felt_ = true;
  }
  if (armed && !isPlane() && !onGround()) {
    // A gust arrives faster than the position controller reacts: about half of the change
    // in wind shows up as ground velocity before it is flown out again.
    vel_ += (air - air_) * 0.5;
  }
  air_ = air;

  fly(dt);

  pos_ += vel_ * dt;
  const double g = ground();
  if (pos_.z <= g) {
    pos_.z = g;
    if (vel_.z < 0.0) {
      vel_.z = 0.0;
    }
    if (!armed || mode == px4_mode::kLand || mode == px4_mode::kRtl) {
      vel_ = {};
    }
  }
  // A hovering vehicle points where it goes. A plane's heading is where its nose is, which
  // in a crosswind is not where it goes (see flyPlaneToward).
  if (!isPlane() && vel_.horizontalNorm() > 0.5) {
    heading_ = std::atan2(vel_.y, vel_.x);
  }

  // PX4 disarms shortly after touching down in an auto mode (COM_DISARM_LAND).
  if (armed && onGround() && vel_.norm() < 0.3 &&
    (mode == px4_mode::kLand || mode == px4_mode::kRtl))
  {
    if (landed_since_ < 0.0) {
      landed_since_ = now;
    }
    if (now - landed_since_ > 1.0) {
      armed = false;
      mode = after_disarm_mode_;
    }
  } else {
    landed_since_ = -1.0;
  }
  if (armed) {
    battery = std::max(0.0, battery - drain_per_s * dt);
  }
  if (isPlane()) {
    saw_fixed_wing = true;
  }
  if (record_trajectory) {
    trajectory.push_back(pos_);
  }
}

VehicleSnapshot KinematicVehicle::snapshot() const
{
  VehicleSnapshot s;
  s.connected = connected;
  s.armed = armed;
  s.mode = mode;
  s.position_valid = connected && position_ok;
  s.position = pos_;
  s.velocity = vel_;
  s.yaw = heading_;
  s.landed = onGround() ? LandedState::OnGround : LandedState::InAir;
  s.vtol = vtol_;
  s.battery = connected ? battery : -1.0;
  s.payload = payload_sensor ? payload_ : PayloadState::Unknown;
  s.weather = reported_weather;
  return s;
}

bool KinematicVehicle::onGround() const {return pos_.z - ground() < 0.05;}

bool KinematicVehicle::isPlane() const
{
  return profile_.vehicle_class == VehicleClass::FixedWing ||
         (profile_.vehicle_class == VehicleClass::Vtol &&
         (vtol_ == VtolState::Fw || vtol_ == VtolState::TransitionToMc));
}

double KinematicVehicle::ground() const {return map_->groundHeight(pos_.x, pos_.y);}

Vec3 KinematicVehicle::windAt(double now) const
{
  // Mean wind plus gusts along it: two slow sines, so runs are repeatable. They swing the
  // wind `gust` either side of the mean (never backwards), so the mean stays the mean and
  // the strongest gust is the mean plus `gust`.
  const double swing =
    0.6 * std::sin(2.0 * kPi * now / 9.0) + 0.4 * std::sin(2.0 * kPi * now / 3.7);
  return wind + wind.normalized() * std::max(-wind.norm(), gust * swing);
}

Vec3 KinematicVehicle::reachable(const Vec3 & ground_velocity) const
{
  // Holding a ground velocity in moving air takes the matching airspeed, and a multirotor
  // has only max_speed of it. Past that it gets what the wind leaves it.
  Vec3 airspeed = ground_velocity - air_;
  const double h = airspeed.horizontalNorm();
  if (h > profile_.max_speed) {
    airspeed.x *= profile_.max_speed / h;
    airspeed.y *= profile_.max_speed / h;
  }
  return airspeed + air_;
}

void KinematicVehicle::flyPlaneToward(const Vec3 & target, double dt)
{
  const Vec3 to = target - pos_;
  const double airspeed = profile_.cruise_speed;
  double want = heading_;
  if (to.horizontalNorm() > 1.0) {
    // Point the nose upwind of the course by the crab angle that cancels the crosswind, as
    // PX4's fixed-wing guidance does with its wind estimate.
    const double course = std::atan2(to.y, to.x);
    const double crosswind = -air_.x * std::sin(course) + air_.y * std::cos(course);
    want = course + std::asin(std::clamp(-crosswind / airspeed, -1.0, 1.0));
  }
  const double max_rate = airspeed / std::max(profile_.turn_radius, 1.0);
  const double err = std::clamp(wrapAngle(want - heading_), -max_rate * dt, max_rate * dt);
  heading_ = wrapAngle(heading_ + err);
  const double vz = std::clamp(to.z * 0.5, -profile_.max_descent_rate, profile_.max_climb_rate);
  vel_ = Vec3{std::cos(heading_) * airspeed, std::sin(heading_) * airspeed, vz} + air_;
}

void KinematicVehicle::approachVelocity(const Vec3 & target_vel, double dt)
{
  Vec3 dv = target_vel - vel_;
  const double max_dv = profile_.max_acceleration * 2.0 * dt;
  if (dv.norm() > max_dv) {
    dv = dv.normalized() * max_dv;
  }
  vel_ += dv;
}

void KinematicVehicle::fly(double dt)
{
  if (!armed) {
    vel_ = {};
    return;
  }
  const double climb = profile_.max_climb_rate;
  const double sink = profile_.max_descent_rate;
  if (mode == px4_mode::kOffboard) {
    if (isPlane()) {
      flyPlaneToward(sp_.position, dt);
      return;
    }
    Vec3 target;
    if (sp_.kind == Setpoint::Kind::Velocity) {
      target = sp_.velocity;
    } else if (sp_.kind == Setpoint::Kind::Position) {
      target = clampVelocity((sp_.position - pos_) * 1.0, profile_.max_speed, climb, sink);
    }
    approachVelocity(reachable(target), dt);
  } else if (mode == px4_mode::kTakeoff) {
    heading_ = runway_heading;
    const double speed = std::max(profile_.min_speed, 5.0) * 1.2;
    vel_ = {std::cos(heading_) * speed, std::sin(heading_) * speed, climb};
    if (!onGround()) {
      vel_ += air_;
    }
    if (pos_.z >= home_.z + profile_.takeoff_altitude) {
      mode = px4_mode::kLoiter;
    }
  } else if (mode == px4_mode::kLoiter) {
    if (isPlane()) {
      heading_ = wrapAngle(heading_ + profile_.cruise_speed / profile_.turn_radius * dt);
      vel_ = Vec3{
        std::cos(heading_) * profile_.cruise_speed, std::sin(heading_) * profile_.cruise_speed,
        0.0} + air_;
    } else {
      approachVelocity(reachable({}), dt);
    }
  } else if (mode == px4_mode::kLand) {
    if (isPlane()) {
      // PX4 fixed-wing Land mode: a circular landing around where it was entered.
      if (pos_.z - ground() > 1.0) {
        heading_ = wrapAngle(heading_ + profile_.cruise_speed / profile_.turn_radius * dt);
        vel_ = Vec3{
          std::cos(heading_) * profile_.cruise_speed, std::sin(heading_) * profile_.cruise_speed,
          -2.0} + air_;
      } else {
        vel_ = {0.0, 0.0, -1.0};  // flare and roll out
      }
    } else {
      approachVelocity(reachable({0.0, 0.0, -1.0}), dt);
    }
  } else if (mode == px4_mode::kRtl) {
    const double d = horizontalDistance(pos_, home_);
    if (isPlane()) {
      // PX4 fixed-wing RTL flies home and then loiters there (RTL_LAND_DELAY -1); it only
      // lands if a mission landing pattern is loaded, which it then flies.
      if (d > profile_.turn_radius) {
        flyPlaneToward({home_.x, home_.y, pos_.z}, dt);
      } else if (fixed_wing_rtl_lands) {
        vel_ = pos_.z - ground() > 1.0 ? Vec3{0.0, 0.0, -3.0} : Vec3{0.0, 0.0, -1.0};
      } else {
        heading_ = wrapAngle(heading_ + profile_.cruise_speed / profile_.turn_radius * dt);
        vel_ = Vec3{
          std::cos(heading_) * profile_.cruise_speed, std::sin(heading_) * profile_.cruise_speed,
          0.0} + air_;
      }
    } else if (d > 1.0) {
      const Vec3 to{home_.x - pos_.x, home_.y - pos_.y, 0.0};
      approachVelocity(reachable(to.normalized() * std::min(profile_.cruise_speed, d)), dt);
    } else {
      approachVelocity(reachable({0.0, 0.0, -1.0}), dt);
    }
  } else {
    // Any other mode (a pilot took over in POSCTL, say): hold.
    approachVelocity(reachable({}), dt);
  }
}

}  // namespace autolivery_core
