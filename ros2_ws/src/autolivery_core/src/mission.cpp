#include "autolivery_core/mission.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace autolivery_core
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string fmt(double v, int precision = 0)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << v;
  return out.str();
}

bool isAirborneState(MissionState s)
{
  switch (s) {
    case MissionState::Takeoff:
    case MissionState::Outbound:
    case MissionState::Descend:
    case MissionState::Release:
    case MissionState::Ascend:
    case MissionState::Return:
    case MissionState::Land:
      return true;
    default:
      return false;
  }
}

}  // namespace

std::string toString(MissionState state)
{
  switch (state) {
    case MissionState::Idle: return "idle";
    case MissionState::Preflight: return "preflight";
    case MissionState::Takeoff: return "takeoff";
    case MissionState::Outbound: return "outbound";
    case MissionState::Descend: return "descend";
    case MissionState::Release: return "release";
    case MissionState::Relaunch: return "relaunch";
    case MissionState::Ascend: return "ascend";
    case MissionState::Return: return "return";
    case MissionState::Land: return "land";
    case MissionState::Complete: return "complete";
    case MissionState::Failed: return "failed";
  }
  return "unknown";
}

Mission::Mission(
  VehicleProfile profile, MissionConfig config, std::shared_ptr<const WorldMap> map,
  PlanFn planner, AvoidFn avoider)
: profile_(std::move(profile)), config_(std::move(config)), map_(std::move(map)),
  planner_(std::move(planner)), avoider_(std::move(avoider))
{
  const auto problems = validate(profile_);
  if (!problems.empty()) {
    std::string text = "vehicle profile '" + profile_.name + "' is invalid:";
    for (const auto & p : problems) {
      text += "\n  - " + p;
    }
    throw std::invalid_argument(text);
  }
  if (!map_) {
    throw std::invalid_argument("mission needs a map");
  }
  if (!planner_) {
    throw std::invalid_argument("mission needs a planner");
  }
  if (!(config_.battery_land < config_.battery_return &&
    config_.battery_return < config_.battery_min_start))
  {
    throw std::invalid_argument(
            "battery thresholds must satisfy land < return < min_start");
  }
}

bool Mission::active() const
{
  return state_ != MissionState::Idle && !finished();
}

std::string Mission::start(
  const DeliveryRequest & request, const VehicleSnapshot & vehicle, double now)
{
  if (active()) {
    return "a mission is already running (" + toString(state_) + ")";
  }
  const WorldMap & map = *map_;
  const Vec3 home = config_.home;
  const Vec3 drop = request.dropoff;
  const double body = profile_.body_radius;
  const double clearance = profile_.planning_clearance;

  if (!drop.isFinite()) {
    return "dropoff is not a finite point";
  }
  if (!map.inBounds(drop)) {
    return "dropoff is outside the geofence";
  }
  if (drop.z < map.groundHeight(drop.x, drop.y) - 0.5) {
    return "dropoff height " + fmt(drop.z, 1) + " m is below the surface there (" +
           fmt(map.groundHeight(drop.x, drop.y), 1) + " m)";
  }
  if (vehicle.armed) {
    return "vehicle is already armed; deliveries start from the ground";
  }
  if (vehicle.battery >= 0.0 && vehicle.battery < config_.battery_min_start) {
    return "battery at " + fmt(vehicle.battery * 100.0) + "%, need " +
           fmt(config_.battery_min_start * 100.0) + "% to start";
  }
  if (config_.payload_feedback && vehicle.payload != PayloadState::Attached) {
    return "no parcel detected in the gripper";
  }
  if (vehicle.position_valid && horizontalDistance(vehicle.position, home) > 5.0) {
    return "vehicle is " + fmt(horizontalDistance(vehicle.position, home), 1) +
           " m from the depot pad; deliveries must start at the depot";
  }

  cruise_z_ = home.z + profile_.cruise_altitude;
  takeoff_top_ = {home.x, home.y, cruise_z_};
  if (map.isBlocked(takeoff_top_, clearance)) {
    return "cruise altitude above the depot is inside an obstacle or the geofence ceiling";
  }
  if (profile_.canHover()) {
    const std::string blocker =
      map.firstBlocker(home + Vec3{0.0, 0.0, body + 0.2}, takeoff_top_, body);
    if (!blocker.empty()) {
      return "the climb out of the depot is blocked by " + blocker;
    }
  }

  PlanResult out;
  PlanResult back;
  double vertical = 2.0 * profile_.cruise_altitude;
  if (profile_.dropoff_method == DropoffMethod::AirDrop) {
    air_drop_target_ = {drop.x, drop.y, drop.z + profile_.release_altitude};
    if (map.isBlocked(air_drop_target_, clearance)) {
      return "the air-drop release point is obstructed";
    }
    const auto dir = chooseAirDropHeading(air_drop_target_, takeoff_top_);
    if (!dir) {
      return "no clear straight approach for the air drop";
    }
    air_drop_dir_ = *dir;
    air_drop_entry_ = air_drop_target_ - air_drop_dir_ * config_.air_drop_approach;
    air_drop_exit_ = air_drop_target_ + air_drop_dir_ * config_.air_drop_overrun;
    out = planner_(takeoff_top_, air_drop_entry_);
    if (out.success && distance(out.path.back(), air_drop_entry_) > clearance) {
      return "planner could not reach the start of the air-drop approach";
    }
    if (out.success) {
      out.path.push_back(air_drop_exit_);
    }
    back = planner_(air_drop_exit_, takeoff_top_);
  } else {
    const bool land = profile_.dropoff_method == DropoffMethod::Land;
    const double release_height = land ? 0.0 : profile_.release_altitude;
    // Arrive over the dropoff at cruise altitude, or high enough above a tall rooftop.
    const double approach_z = std::max(cruise_z_, drop.z + release_height + 10.0);
    above_drop_ = {drop.x, drop.y, approach_z};
    release_point_ = {drop.x, drop.y, drop.z + release_height};
    if (map.isBlocked(above_drop_, clearance)) {
      return "the airspace above the dropoff is blocked";
    }
    const Vec3 bottom = land ? drop + Vec3{0.0, 0.0, body + 0.2} : release_point_;
    const std::string blocker = map.firstBlocker(above_drop_, bottom, body);
    if (!blocker.empty()) {
      return "the descent to the dropoff is blocked by " + blocker;
    }
    out = planner_(takeoff_top_, above_drop_);
    if (out.success && distance(out.path.back(), above_drop_) > profile_.acceptance_radius) {
      return "planner could not reach the point above the dropoff";
    }
    back = planner_(above_drop_, takeoff_top_);
    vertical += 2.0 * (approach_z - release_point_.z);
  }
  if (!out.success) {
    return "no route to the dropoff: " + out.message;
  }
  if (!back.success) {
    return "no route back to the depot: " + back.message;
  }
  if (distance(back.path.back(), takeoff_top_) > profile_.acceptance_radius) {
    return "planner could not reach the point above the depot";
  }

  const double route = pathLength(out.path) + pathLength(back.path) + vertical;
  if (route > profile_.max_range) {
    return "route is " + fmt(route) + " m but " + profile_.name + " range is " +
           fmt(profile_.max_range) + " m";
  }

  outbound_path_ = std::move(out.path);
  return_path_ = std::move(back.path);
  request_ = request;
  result_ = MissionResult{};
  follower_.reset({});
  leg_ = Leg::None;
  ready_since_ = -1.0;
  settle_since_ = -1.0;
  first_release_ = -1.0;
  last_release_ = -1.0;
  air_drop_attempt_ = 0;
  pass_missed_ = false;
  aborted_ = false;
  stand_down_ = false;
  land_via_rtl_ = false;
  avoiding_ = false;
  threat_.clear();
  wanted_mode_.clear();
  reached_mode_.clear();
  pending_cancel_.reset();
  last_update_ = -1.0;
  last_position_.reset();
  was_armed_ = vehicle.armed;
  hold_xy_ = vehicle.position_valid ? vehicle.position : home;

  transition(
    MissionState::Preflight, now,
    "delivery to " + (request.label.empty() ? std::string("dropoff") : request.label) +
    " accepted, route " + fmt(route) + " m");
  return {};
}

void Mission::cancel(const std::string & reason)
{
  if (active()) {
    pending_cancel_ = reason;
  }
}

std::vector<std::string> Mission::takeEvents()
{
  std::vector<std::string> out;
  out.swap(events_);
  return out;
}

MissionStatus Mission::status() const
{
  MissionStatus s;
  s.state = state_;
  s.leg = leg_ == Leg::Outbound ? "outbound" : leg_ == Leg::Return ? "return" : "";
  if (state_ == MissionState::Outbound || state_ == MissionState::Return) {
    s.leg_remaining = last_progress_.remaining;
    s.waypoint = last_progress_.segment + 1;
    s.waypoint_count = follower_.path().size();
  }
  s.avoiding = avoiding_;
  s.threat = threat_;
  s.detail = detail_;
  return s;
}

MissionCommand Mission::update(const VehicleSnapshot & v, double now)
{
  MissionCommand cmd;

  // Flight statistics.
  if (last_update_ >= 0.0 && v.armed) {
    result_.flight_time += now - last_update_;
  }
  last_update_ = now;
  if (v.position_valid) {
    if (last_position_ && v.armed) {
      result_.distance_flown += distance(*last_position_, v.position);
    }
    last_position_ = v.position;
  }

  if (!active()) {
    // Standing down after a ground abort: make sure the motors are off, then go quiet.
    if (state_ == MissionState::Failed && stand_down_ && v.armed &&
      v.landed == LandedState::OnGround)
    {
      cmd.armed = false;
    }
    was_armed_ = v.armed;
    return cmd;
  }

  if (pending_cancel_) {
    const std::string reason = *pending_cancel_;
    pending_cancel_.reset();
    abortToReturn(reason, v, now);
  }

  if (active() && checkExternalTakeover(v, cmd)) {
    was_armed_ = v.armed;
    return cmd;
  }
  // PX4 disarms by itself after an AUTO.LAND touchdown; anywhere else a disarm while the
  // mission is flying means the vehicle was killed or fell out of the sky.
  const bool disarm_expected = state_ == MissionState::Land ||
    (profile_.dropoff_method == DropoffMethod::Land &&
    (state_ == MissionState::Descend || state_ == MissionState::Release));
  if (active() && isAirborneState(state_) && !disarm_expected && was_armed_ && !v.armed) {
    fail("vehicle disarmed in flight", now);
    was_armed_ = v.armed;
    return cmd;
  }
  if (active()) {
    checkBattery(v, now, cmd);
  }

  switch (state_) {
    case MissionState::Preflight: doPreflight(v, now, cmd, false); break;
    case MissionState::Relaunch: doPreflight(v, now, cmd, true); break;
    case MissionState::Takeoff: doTakeoff(v, now, cmd); break;
    case MissionState::Outbound:
    case MissionState::Return: doLeg(v, now, cmd); break;
    case MissionState::Descend: doDescend(v, now, cmd); break;
    case MissionState::Release: doRelease(v, now, cmd); break;
    case MissionState::Ascend: doAscend(v, now, cmd); break;
    case MissionState::Land: doLand(v, now, cmd); break;
    default: break;
  }

  if (!active()) {
    // A handler just finished or failed the mission; say nothing to the vehicle.
    MissionCommand quiet;
    if (state_ == MissionState::Failed && stand_down_ && v.armed &&
      v.landed == LandedState::OnGround)
    {
      quiet.armed = false;
    }
    cmd = quiet;
  }

  // Track which mode we asked for and whether the vehicle got there, for takeover detection.
  if (cmd.mode.empty()) {
    wanted_mode_.clear();
    reached_mode_.clear();
  } else if (cmd.mode != wanted_mode_) {
    wanted_mode_ = cmd.mode;
    reached_mode_ = v.mode == cmd.mode ? cmd.mode : std::string{};
  } else if (v.mode == wanted_mode_) {
    reached_mode_ = wanted_mode_;
  }
  was_armed_ = v.armed;
  return cmd;
}

bool Mission::checkExternalTakeover(const VehicleSnapshot & v, MissionCommand & cmd)
{
  if (reached_mode_.empty() || v.mode == reached_mode_) {
    return false;
  }
  // A vehicle that is disarmed on the ground cannot be taken over in any sense that matters,
  // and PX4 changes mode by itself there: when AUTO.LAND disarms it, commander reverts to the
  // last mode it was commanded into that allows arming (OFFBOARD, for us).
  if (!v.armed && v.landed == LandedState::OnGround) {
    return false;
  }
  // PX4 drops into Hold on its own when a fixed-wing takeoff completes; that is expected.
  if (reached_mode_ == px4_mode::kTakeoff && v.mode == px4_mode::kLoiter) {
    return false;
  }
  const std::string from = reached_mode_;
  fail(
    "flight mode changed from " + from + " to " + (v.mode.empty() ? "unknown" : v.mode) +
    " outside the mission; control left with the pilot/autopilot", last_update_);
  cmd = MissionCommand{};
  wanted_mode_.clear();
  reached_mode_.clear();
  return true;
}

bool Mission::checkBattery(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  (void)cmd;
  if (v.battery < 0.0) {
    return false;
  }
  const std::string pct = fmt(v.battery * 100.0) + "%";
  if (v.battery < config_.battery_land && isAirborneState(state_) &&
    state_ != MissionState::Land)
  {
    emergencyLand("battery critical at " + pct + ", landing immediately", now);
    return true;
  }
  if (v.battery < config_.battery_return && !aborted_) {
    switch (state_) {
      case MissionState::Takeoff:
      case MissionState::Outbound:
      case MissionState::Descend:
        abortToReturn("battery low at " + pct + ", abandoning delivery", v, now);
        return true;
      case MissionState::Release:
        // Finish a release that is under way; the parcel is lighter than the risk of
        // carrying it home on a low battery.
        return false;
      case MissionState::Relaunch:
        // On the ground at the dropoff: staying there is safer than taking off.
        stand_down_ = true;
        fail("battery low at " + pct + " at the dropoff; staying landed", now);
        return true;
      default:
        return false;
    }
  }
  return false;
}

void Mission::doPreflight(
  const VehicleSnapshot & v, double now, MissionCommand & cmd, bool relaunch)
{
  const bool fixed_wing_takeoff =
    !relaunch && profile_.vehicle_class == VehicleClass::FixedWing;

  if (now - state_since_ > config_.preflight_timeout) {
    std::string why = "vehicle not ready";
    if (v.connected && v.position_valid) {
      why = v.mode != (fixed_wing_takeoff ? px4_mode::kTakeoff : px4_mode::kOffboard) ?
        "vehicle did not accept " +
        std::string(fixed_wing_takeoff ? px4_mode::kTakeoff : px4_mode::kOffboard) + " mode" :
        "vehicle did not arm";
    }
    stand_down_ = true;
    fail(why + " within " + fmt(config_.preflight_timeout) + " s", now);
    return;
  }
  if (!v.connected || !v.position_valid) {
    ready_since_ = -1.0;
    detail_ = "waiting for vehicle connection and position estimate";
    return;
  }

  // Stream a setpoint the vehicle can accept as-is before asking for OFFBOARD: PX4 refuses
  // the mode switch until it has seen setpoints for a while, and arms straight into it.
  if (fixed_wing_takeoff) {
    cmd.setpoint = holdAt(outbound_path_.size() > 1 ? outbound_path_[1] : takeoff_top_);
  } else {
    cmd.setpoint = holdAt(v.position);
  }
  if (profile_.vehicle_class == VehicleClass::Vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  if (ready_since_ < 0.0) {
    ready_since_ = now;
  }
  if (now - ready_since_ < config_.offboard_warmup) {
    detail_ = "streaming setpoints before mode switch";
    return;
  }

  cmd.mode = fixed_wing_takeoff ? px4_mode::kTakeoff : px4_mode::kOffboard;
  // Arm only once the mode is in: arming in a manual mode on a vehicle without RC is refused,
  // and arming first then switching gives PX4 a window to auto-disarm.
  if (v.mode == cmd.mode) {
    cmd.armed = true;
  }
  detail_ = v.mode != cmd.mode ? "requesting " + cmd.mode : "arming";
  if (v.armed && v.mode == cmd.mode) {
    hold_xy_ = v.position;
    if (relaunch) {
      transition(MissionState::Ascend, now, "relaunched from the dropoff");
    } else {
      transition(MissionState::Takeoff, now, "armed in " + cmd.mode);
    }
  }
}

void Mission::doTakeoff(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  cmd.armed = true;
  if (now - state_since_ > config_.takeoff_timeout) {
    abortToReturn("takeoff did not reach cruise altitude in time", v, now);
    return;
  }

  if (profile_.vehicle_class == VehicleClass::FixedWing) {
    const Vec3 first = outbound_path_.size() > 1 ? outbound_path_[1] : takeoff_top_;
    cmd.setpoint = holdAt(first);
    const bool high_enough =
      v.position.z >= config_.home.z + profile_.takeoff_altitude - config_.altitude_tolerance;
    cmd.mode = high_enough ? px4_mode::kOffboard : px4_mode::kTakeoff;
    if (v.mode == px4_mode::kLoiter && !high_enough) {
      // Takeoff finished below our threshold (MIS_TAKEOFF_ALT lower than takeoff_altitude):
      // take over anyway rather than time out circling.
      cmd.mode = px4_mode::kOffboard;
    }
    if (v.mode == px4_mode::kOffboard) {
      startLeg(Leg::Outbound, outbound_path_, now);
    }
    detail_ = "climbing out";
    return;
  }

  if (profile_.vehicle_class == VehicleClass::Vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  cmd.mode = px4_mode::kOffboard;
  const Vec3 target{hold_xy_.x, hold_xy_.y, cruise_z_};
  cmd.setpoint = holdAt(target);
  detail_ = "climbing to " + fmt(profile_.cruise_altitude) + " m";
  if (std::abs(v.position.z - cruise_z_) < config_.altitude_tolerance &&
    horizontalDistance(v.position, target) < 2.0 * config_.position_tolerance)
  {
    startLeg(Leg::Outbound, outbound_path_, now);
  }
}

void Mission::doLeg(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  cmd.mode = px4_mode::kOffboard;
  cmd.armed = true;

  if (now > leg_deadline_) {
    if (leg_ == Leg::Outbound) {
      abortToReturn("outbound leg took too long", v, now);
    } else {
      emergencyLand("return leg took too long, landing where it is safe to descend", now);
    }
    return;
  }

  const bool plane = flyingAsPlane(v);
  const double acceptance = plane ?
    std::max(profile_.acceptance_radius, profile_.turn_radius) : profile_.acceptance_radius;
  const double lookahead = plane ?
    std::max(config_.lookahead, 2.5 * profile_.turn_radius) : config_.lookahead;
  const PathFollower::Progress p = follower_.update(v.position, lookahead, acceptance);
  last_progress_ = p;

  if (profile_.vehicle_class == VehicleClass::Vtol) {
    const double margin = config_.vtol_transition_margin;
    const double along = follower_.totalLength() - p.remaining;
    const bool long_leg = follower_.totalLength() > 2.0 * margin + 100.0;
    cmd.vtol = long_leg && along > margin && p.remaining > margin ?
      VtolRequest::Fw : VtolRequest::Mc;
  }

  const bool air_drop = leg_ == Leg::Outbound &&
    profile_.dropoff_method == DropoffMethod::AirDrop;
  if (air_drop && p.final_segment && first_release_ < 0.0 && !pass_missed_) {
    bool on_line = false;
    if (airDropTrigger(v, &on_line)) {
      if (on_line) {
        cmd.release = true;
        first_release_ = last_release_ = now;
        event("parcel released over " + request_.label);
      } else {
        pass_missed_ = true;
        event("predicted impact too far off the target line, not releasing on this pass");
      }
    }
  }
  if (air_drop && first_release_ >= 0.0 && config_.payload_feedback &&
    v.payload == PayloadState::Attached && now - last_release_ >= config_.release_retry_period)
  {
    // Released on the pass but the sensor still sees the parcel: the gripper stuck. Retrying
    // now would drop it off-target, so leave it for the go-around.
    pass_missed_ = true;
  }

  if (p.finished) {
    const Vec3 end = follower_.path().back();
    cmd.setpoint = holdAt(end);
    if (leg_ == Leg::Return) {
      transition(MissionState::Land, now, "arrived over the depot");
      land_target_ = end;
      return;
    }
    if (!air_drop) {
      transition(MissionState::Descend, now, "arrived over " + request_.label);
      return;
    }
    const bool dropped = config_.payload_feedback ?
      v.payload == PayloadState::Released : first_release_ >= 0.0;
    if (dropped) {
      result_.delivered = true;
      startLeg(Leg::Return, return_path_, now);
      return;
    }
    if (++air_drop_attempt_ < config_.air_drop_attempts) {
      PlanResult around = planner_(v.position, air_drop_entry_);
      if (around.success) {
        around.path.push_back(air_drop_exit_);
        first_release_ = -1.0;
        event("going around for another air-drop pass");
        startLeg(Leg::Outbound, std::move(around.path), now);
        return;
      }
    }
    abortToReturn("air drop missed on every pass", v, now);
    return;
  }

  if (plane) {
    Vec3 target = p.carrot;
    if (avoider_) {
      const Vec3 desired = (target - v.position).normalized() * profile_.cruise_speed;
      const AvoidanceResult a = avoider_(v.position, desired);
      if (a.modified && a.velocity.norm() > 0.1) {
        target = v.position + a.velocity.normalized() * lookahead;
      }
      if (a.modified != avoiding_) {
        event(a.modified ? "avoiding " + a.threat : "avoidance cleared");
      }
      avoiding_ = a.modified;
      threat_ = a.threat;
    }
    cmd.setpoint = holdAt(target);
    detail_ = "cruising, " + fmt(p.remaining) + " m to go";
    return;
  }

  // Multicopter: velocity toward the carrot, braking so the vehicle can stop at the end.
  const double brake = 0.5 * profile_.max_acceleration;
  double speed = std::min(
    profile_.cruise_speed,
    std::sqrt(2.0 * brake * std::max(0.0, p.remaining)));
  speed = std::max(speed, 0.5);
  Vec3 to_carrot = p.carrot - v.position;
  if (to_carrot.norm() < 0.1) {
    to_carrot = follower_.path().back() - v.position;
  }
  Vec3 desired = clampVelocity(
    to_carrot.normalized() * speed, profile_.max_speed, profile_.max_climb_rate,
    profile_.max_descent_rate);
  if (avoider_) {
    const AvoidanceResult a = avoider_(v.position, desired);
    if (a.modified != avoiding_) {
      event(a.modified ? "avoiding " + a.threat : "avoidance cleared");
    }
    avoiding_ = a.modified;
    threat_ = a.threat;
    desired = a.velocity;
  }
  cmd.setpoint.kind = Setpoint::Kind::Velocity;
  cmd.setpoint.position = p.carrot;
  cmd.setpoint.velocity = desired;
  cmd.setpoint.yaw = desired.horizontalNorm() > 1.0 ? std::atan2(desired.y, desired.x) : kNaN;
  detail_ = (avoiding_ ? "avoiding " + threat_ + ", " : std::string("cruising, ")) +
    fmt(p.remaining) + " m to go";
}

void Mission::doDescend(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  const bool vtol = profile_.vehicle_class == VehicleClass::Vtol;
  if (vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  if (vtol && v.vtol != VtolState::Mc) {
    // Back-transition still in progress: hold over the dropoff until it is a multicopter.
    cmd.mode = px4_mode::kOffboard;
    cmd.armed = true;
    cmd.setpoint = holdAt(above_drop_);
    detail_ = "waiting for multicopter mode";
    if (now - state_since_ > config_.descend_timeout) {
      abortToReturn("VTOL did not transition to multicopter over the dropoff", v, now);
    }
    return;
  }

  if (profile_.dropoff_method == DropoffMethod::Land) {
    if (!landing_committed_ && !settleBeforeLanding(v, above_drop_, now, cmd)) {
      detail_ = "stopping over " + request_.label + " before landing";
      return;
    }
    cmd.mode = px4_mode::kLand;
    detail_ = "landing at " + request_.label;
    if (v.landed == LandedState::OnGround && !v.armed) {
      transition(MissionState::Release, now, "landed at " + request_.label);
      return;
    }
    if (now - state_since_ > config_.land_timeout) {
      abortToReturn("landing at the dropoff did not complete", v, now);
    }
    return;
  }

  cmd.mode = px4_mode::kOffboard;
  cmd.armed = true;
  cmd.setpoint = holdAt(release_point_);
  detail_ = "descending to " + fmt(profile_.release_altitude, 1) + " m over " + request_.label;
  if (settled(v, release_point_, now)) {
    transition(MissionState::Release, now, "holding over " + request_.label);
    return;
  }
  if (now - state_since_ > config_.descend_timeout) {
    abortToReturn("could not settle over the dropoff", v, now);
  }
}

void Mission::doRelease(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  const bool landed = profile_.dropoff_method == DropoffMethod::Land;
  if (!landed) {
    cmd.mode = px4_mode::kOffboard;
    cmd.armed = true;
    cmd.setpoint = holdAt(release_point_);
    if (profile_.vehicle_class == VehicleClass::Vtol) {
      cmd.vtol = VtolRequest::Mc;
    }
  }

  const bool confirmed = config_.payload_feedback ?
    v.payload == PayloadState::Released :
    first_release_ >= 0.0 && now - first_release_ >= config_.release_duration;
  if (confirmed) {
    result_.delivered = true;
    event("parcel delivered to " + request_.label);
    transition(landed ? MissionState::Relaunch : MissionState::Ascend, now, "parcel released");
    return;
  }
  // Without a sensor the release is confirmed by time alone, and a winch takes a while to
  // pay out; never time out before the release could possibly have finished.
  const double timeout = std::max(config_.release_timeout, config_.release_duration + 5.0);
  if (now - state_since_ > timeout) {
    abortToReturn("the parcel did not release", v, now);
    return;
  }
  if (first_release_ < 0.0 ||
    (config_.payload_feedback && now - last_release_ >= config_.release_retry_period))
  {
    cmd.release = true;
    if (first_release_ < 0.0) {
      first_release_ = now;
    }
    last_release_ = now;
  }
  detail_ = "releasing parcel";
}

void Mission::doAscend(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  cmd.mode = px4_mode::kOffboard;
  cmd.armed = true;
  if (profile_.vehicle_class == VehicleClass::Vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  cmd.setpoint = holdAt(above_drop_);
  detail_ = "climbing back to cruise altitude";
  if (std::abs(v.position.z - above_drop_.z) < config_.altitude_tolerance) {
    startLeg(Leg::Return, return_path_, now);
    return;
  }
  if (now - state_since_ > config_.takeoff_timeout) {
    emergencyLand("could not climb back to cruise altitude", now);
  }
}

void Mission::doLand(const VehicleSnapshot & v, double now, MissionCommand & cmd)
{
  // Planes: with a landing pattern loaded, RTL flies it -- a straight-in approach to the
  // runway, which is how PX4 lands fixed-wings reliably. Without one, AUTO.LAND is PX4's
  // circular landing where Land mode is entered (over the airstrip once the return leg is
  // flown). A fixed-wing RTL without a pattern only loiters over home (RTL_LAND_DELAY -1), so
  // if RTL was the no-route fallback, the mission hands over to AUTO.LAND once it is there.
  const bool fixed_wing = profile_.vehicle_class == VehicleClass::FixedWing;
  bool rtl = land_via_rtl_ || (fixed_wing && config_.fw_mission_landing);
  if (rtl && fixed_wing && !config_.fw_mission_landing &&
    horizontalDistance(v.position, config_.home) < 3.0 * profile_.turn_radius)
  {
    rtl = false;
  }
  if (!rtl && !fixed_wing && !landing_committed_ &&
    !settleBeforeLanding(v, land_target_, now, cmd))
  {
    detail_ = "stopping over the depot before landing";
    return;
  }
  // From here the autopilot flies. No setpoints are streamed in AUTO modes: PX4 ignores them,
  // and an active stream is what lets it (or a pilot's switch) drop back into OFFBOARD.
  cmd.mode = rtl ? px4_mode::kRtl : px4_mode::kLand;
  if (profile_.vehicle_class == VehicleClass::Vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  detail_ = rtl ? "returning home (RTL)" : fixed_wing ? "landing (circling down)" : "landing";
  if (v.landed == LandedState::OnGround && !v.armed) {
    result_.success = result_.delivered && !aborted_;
    if (result_.success) {
      result_.message = "delivered to " + request_.label + " and back at the depot";
    } else if (result_.message.empty()) {
      result_.message = "landed without completing the delivery";
    }
    transition(MissionState::Complete, now, result_.message);
    return;
  }
  if (now - state_since_ > config_.land_timeout) {
    fail("landing not confirmed within " + fmt(config_.land_timeout) + " s", now);
  }
}

void Mission::abortToReturn(const std::string & reason, const VehicleSnapshot & v, double now)
{
  if (!aborted_) {
    aborted_ = true;
    result_.message = reason;
  }
  event("abort: " + reason);
  switch (state_) {
    case MissionState::Preflight:
      stand_down_ = true;
      fail(reason, now);
      return;
    case MissionState::Takeoff:
      // Still climbing out of the depot: the nearest safe place is straight below.
      transition(MissionState::Land, now, reason);
      landing_committed_ = true;
      return;
    case MissionState::Outbound: {
        PlanResult home = planner_(v.position, takeoff_top_);
        if (home.success) {
          startLeg(Leg::Return, std::move(home.path), now);
        } else {
          // Last resort: hand the flight home to the autopilot's RTL.
          land_via_rtl_ = true;
          transition(MissionState::Land, now, reason + "; no planned route home, using RTL");
        }
        return;
      }
    case MissionState::Descend:
    case MissionState::Release:
      if (profile_.dropoff_method == DropoffMethod::Land) {
        // Landing (or landed) at the dropoff: take off again and go home.
        transition(
          v.armed ? MissionState::Ascend : MissionState::Relaunch, now, reason);
      } else {
        transition(MissionState::Ascend, now, reason);
      }
      return;
    case MissionState::Relaunch:
      stand_down_ = true;
      fail(reason, now);
      return;
    default:
      // Ascend, Return and Land are already on their way home.
      return;
  }
}

void Mission::emergencyLand(const std::string & reason, double now)
{
  aborted_ = true;
  result_.message = reason;
  event("emergency: " + reason);
  transition(MissionState::Land, now, reason);
  landing_committed_ = true;  // no settling: land now, where it is
}

void Mission::fail(const std::string & reason, double now)
{
  result_.success = false;
  result_.message = reason;
  transition(MissionState::Failed, now, reason);
}

void Mission::transition(MissionState next, double now, const std::string & why)
{
  std::string text = toString(state_) + " -> " + toString(next);
  if (!why.empty()) {
    text += ": " + why;
  }
  event(text);
  state_ = next;
  state_since_ = now;
  settle_since_ = -1.0;
  if (next == MissionState::Release) {
    first_release_ = -1.0;
    last_release_ = -1.0;
  }
  if (next == MissionState::Relaunch) {
    ready_since_ = -1.0;
  }
  if (next == MissionState::Land || next == MissionState::Descend) {
    landing_committed_ = false;
  }
  if (next != MissionState::Outbound && next != MissionState::Return) {
    avoiding_ = false;
    threat_.clear();
  }
}

void Mission::startLeg(Leg leg, Path path, double now)
{
  leg_ = leg;
  follower_.reset(std::move(path));
  last_progress_ = PathFollower::Progress{};
  last_progress_.remaining = follower_.totalLength();
  leg_deadline_ = legDeadline(follower_.totalLength(), now);
  pass_missed_ = false;
  transition(
    leg == Leg::Outbound ? MissionState::Outbound : MissionState::Return, now,
    fmt(follower_.totalLength()) + " m, " + std::to_string(follower_.path().size()) +
    " waypoints");
}

void Mission::event(const std::string & text)
{
  events_.push_back(text);
  detail_ = text;
}

bool Mission::flyingAsPlane(const VehicleSnapshot & v) const
{
  switch (profile_.vehicle_class) {
    case VehicleClass::FixedWing: return true;
    case VehicleClass::Multicopter: return false;
    case VehicleClass::Vtol:
      return v.vtol == VtolState::Fw || v.vtol == VtolState::TransitionToFw ||
             v.vtol == VtolState::TransitionToMc;
  }
  return false;
}

bool Mission::settleBeforeLanding(
  const VehicleSnapshot & v, const Vec3 & target, double now, MissionCommand & cmd)
{
  // AUTO.LAND comes straight down from wherever it starts, so hold over the spot until the
  // vehicle has stopped first; otherwise the landing error is the whole acceptance radius
  // plus the braking distance. Give up waiting (and land anyway) rather than hover forever.
  constexpr double kSettleTimeout = 30.0;
  cmd.mode = px4_mode::kOffboard;
  cmd.armed = true;
  if (profile_.vehicle_class == VehicleClass::Vtol) {
    cmd.vtol = VtolRequest::Mc;
  }
  cmd.setpoint = holdAt(target);
  if (settled(v, target, now) || now - state_since_ > kSettleTimeout) {
    landing_committed_ = true;
    cmd = MissionCommand{};
    return true;
  }
  return false;
}

bool Mission::settled(const VehicleSnapshot & v, const Vec3 & target, double now)
{
  if (distance(v.position, target) <= config_.position_tolerance &&
    v.velocity.norm() <= config_.settle_speed)
  {
    if (settle_since_ < 0.0) {
      settle_since_ = now;
    }
    return now - settle_since_ >= config_.settle_time;
  }
  settle_since_ = -1.0;
  return false;
}

Setpoint Mission::holdAt(const Vec3 & p, double yaw) const
{
  Setpoint s;
  s.kind = Setpoint::Kind::Position;
  s.position = p;
  s.yaw = yaw;
  return s;
}

double Mission::legDeadline(double length, double now) const
{
  return now + length / profile_.cruise_speed * config_.leg_timeout_factor +
         config_.leg_timeout_margin;
}

bool Mission::airDropTrigger(const VehicleSnapshot & v, bool * on_line) const
{
  const double h = v.position.z - request_.dropoff.z;
  if (h <= 0.5) {
    return false;
  }
  // Drag-free ballistic fall from the current state. Parachuted parcels need a drag model
  // (and a much shorter lead); this is correct for compact, dense parcels and for Gazebo,
  // which applies no aerodynamic drag to the parcel.
  const double vz = v.velocity.z;
  const double t_fall = (vz + std::sqrt(vz * vz + 2.0 * kGravity * h)) / kGravity;
  const double t = t_fall + config_.air_drop_latency;
  const Vec3 impact = v.position.horizontal() + v.velocity.horizontal() * t;
  const Vec3 rel = impact - air_drop_target_.horizontal();
  const double along = rel.dot(air_drop_dir_);
  const double cross = (rel - air_drop_dir_ * along).horizontalNorm();
  const double ground_speed_on_line = v.velocity.horizontal().dot(air_drop_dir_);
  *on_line = cross <= config_.air_drop_max_cross_track &&
    ground_speed_on_line > 0.5 * v.velocity.horizontalNorm();
  return along >= 0.0;
}

std::optional<Vec3> Mission::chooseAirDropHeading(const Vec3 & target, const Vec3 & from) const
{
  Vec3 preferred = (target - from).horizontal();
  if (preferred.norm() < 1.0) {
    preferred = {1.0, 0.0, 0.0};
  }
  const double base = std::atan2(preferred.y, preferred.x);
  constexpr int kHeadings = 16;
  // Try headings in order of how little they deviate from the direction of travel.
  for (int k = 0; k <= kHeadings / 2; ++k) {
    for (int sign : {1, -1}) {
      if (k == 0 && sign < 0) {
        continue;
      }
      if (k == kHeadings / 2 && sign < 0) {
        continue;
      }
      const double h = base + sign * k * 2.0 * kPi / kHeadings;
      const Vec3 dir{std::cos(h), std::sin(h), 0.0};
      const Vec3 entry = target - dir * config_.air_drop_approach;
      const Vec3 exit = target + dir * config_.air_drop_overrun;
      if (map_->segmentClear(entry, exit, profile_.planning_clearance)) {
        return dir;
      }
    }
  }
  return std::nullopt;
}

}  // namespace autolivery_core
