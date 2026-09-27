// The delivery mission: depot -> dropoff -> depot, as a state machine with no I/O.
//
//   Idle -> Preflight -> Takeoff -> Outbound -> Descend -> Release -> Ascend -> Return -> Land
//                                     |                      |                            |
//                                     |    (land dropoff) Relaunch                   Complete
//                                     +-- (air drop: release in flight, then Return)
//
// Each update() takes a snapshot of the vehicle and returns what the vehicle should be doing:
// the setpoint to stream, the flight mode and arming state wanted, a VTOL transition, and
// whether to fire the gripper. The ROS node turns that into MAVROS traffic; the unit tests
// turn it into a kinematic fake vehicle. Nothing in here knows about either.
//
// Safety rules this class is built around:
//
//  * The autopilot outranks the mission. If the flight mode leaves OFFBOARD without the
//    mission asking -- pilot takeover, PX4 failsafe (geofence, RC/datalink loss, low battery)
//    -- the mission fails immediately, stops streaming setpoints and stops requesting modes.
//    Streaming stale setpoints after a takeover is how a pilot's later switch back to
//    OFFBOARD turns into the vehicle lunging for a waypoint from five minutes ago.
//  * Everything that can fail has a deadline. A mission never waits forever for a mode
//    change, a climb, a release, or a landing.
//  * Aborts go home by a planned path, not a straight line: a straight-line RTL in a city
//    goes through buildings unless RTL_RETURN_ALT clears every roof. Only when replanning
//    itself fails is the autopilot's RTL used as the last resort.
//  * A mission is only accepted if the whole route -- takeoff corridor, outbound, descent,
//    return -- is planned, clear, and within range before the motors start.

#ifndef AUTOLIVERY_CORE__MISSION_HPP_
#define AUTOLIVERY_CORE__MISSION_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/path_follower.hpp"
#include "autolivery_core/planner.hpp"
#include "autolivery_core/vehicle_profile.hpp"
#include "autolivery_core/velocity_obstacle.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

// PX4 custom mode names as MAVROS reports and accepts them.
namespace px4_mode
{
constexpr const char * kOffboard = "OFFBOARD";
constexpr const char * kTakeoff = "AUTO.TAKEOFF";
constexpr const char * kLand = "AUTO.LAND";
constexpr const char * kRtl = "AUTO.RTL";
constexpr const char * kLoiter = "AUTO.LOITER";
}  // namespace px4_mode

enum class MissionState
{
  Idle,
  Preflight,
  Takeoff,
  Outbound,
  Descend,
  Release,
  Relaunch,
  Ascend,
  Return,
  Land,
  Complete,
  Failed,
};

std::string toString(MissionState state);

enum class LandedState {Unknown, OnGround, InAir, Takeoff, Landing};
enum class VtolState {Undefined, Mc, Fw, TransitionToFw, TransitionToMc};
enum class PayloadState {Unknown, Attached, Released};
enum class VtolRequest {None, Mc, Fw};

struct VehicleSnapshot
{
  bool connected{false};
  bool armed{false};
  std::string mode;
  bool position_valid{false};
  Vec3 position;             // world frame
  Vec3 velocity;             // world frame
  double yaw{0.0};           // rad, ENU (0 = east, CCW positive)
  LandedState landed{LandedState::Unknown};
  VtolState vtol{VtolState::Undefined};
  double battery{-1.0};      // remaining fraction in [0, 1]; negative when unknown
  PayloadState payload{PayloadState::Unknown};
};

struct Setpoint
{
  enum class Kind {None, Position, Velocity};
  Kind kind{Kind::None};
  Vec3 position;             // world frame
  Vec3 velocity;             // world frame
  double yaw{std::numeric_limits<double>::quiet_NaN()};  // NaN = hold current heading
};

struct MissionCommand
{
  Setpoint setpoint;
  std::string mode;              // wanted flight mode; empty = no opinion
  std::optional<bool> armed;     // wanted arming state; nullopt = no opinion
  VtolRequest vtol{VtolRequest::None};
  bool release{false};           // fire one release attempt now
};

struct MissionConfig
{
  Vec3 home;                          // depot pad surface, world frame

  double offboard_warmup{1.5};        // s of setpoints before asking for OFFBOARD (PX4 wants >1 s)
  double position_tolerance{1.0};     // m
  double altitude_tolerance{0.8};     // m
  double settle_speed{0.5};           // m/s, "stopped" for release purposes
  double settle_time{2.0};            // s the vehicle must be stopped before releasing
  double lookahead{8.0};              // m, path-follower carrot distance (multirotor)

  bool payload_feedback{false};       // a sensor reports PayloadState; otherwise trust timing
  double release_duration{1.5};       // s after the release command without feedback
  double release_retry_period{4.0};   // s between release attempts with feedback
  double release_timeout{20.0};       // s before a stuck parcel aborts the delivery

  double battery_return{0.30};        // below this, abandon the delivery and go home
  double battery_land{0.15};          // below this, land now wherever the vehicle is
  double battery_min_start{0.60};     // refuse to start below this

  double vtol_transition_margin{150.0};  // m at each end of a leg flown as a multicopter

  // The autopilot holds a mission landing pattern for this depot (PX4 RTL_TYPE 1 flies it on
  // RTL), so fixed-wings finish with RTL: a straight-in approach to the runway. Without one
  // they use AUTO.LAND, which PX4 flies as a circular landing where Land mode is entered.
  bool fw_mission_landing{false};

  double air_drop_approach{200.0};    // m of straight, level approach before the target
  double air_drop_overrun{150.0};     // m flown past the target before turning for home
  double air_drop_latency{0.25};      // s from release command to the parcel actually falling
  double air_drop_max_cross_track{10.0};  // m, predicted impact further off-line = go around
  int air_drop_attempts{2};

  double preflight_timeout{60.0};     // s for the vehicle to become ready, enter mode and arm
  double takeoff_timeout{90.0};
  double descend_timeout{60.0};
  double land_timeout{240.0};
  double leg_timeout_factor{2.5};     // leg deadline = length / cruise_speed * factor + margin
  double leg_timeout_margin{60.0};
};

struct DeliveryRequest
{
  Vec3 dropoff;        // world frame; z is the surface the parcel ends up on
  std::string label;   // for logs and status
};

struct MissionResult
{
  bool delivered{false};
  bool success{false};        // delivered and back on the ground at the depot
  std::string message;
  double flight_time{0.0};    // s armed
  double distance_flown{0.0};  // m
};

struct MissionStatus
{
  MissionState state{MissionState::Idle};
  std::string leg;             // "outbound", "return", ... for humans
  double leg_remaining{0.0};   // m left on the current path leg
  std::size_t waypoint{0};
  std::size_t waypoint_count{0};
  bool avoiding{false};
  std::string threat;
  std::string detail;          // last notable event
};

// Plans a smoothed, clear path between two world points.
using PlanFn = std::function<PlanResult (const Vec3 & from, const Vec3 & to)>;
// Bends a desired velocity around dynamic obstacles; see VelocityObstacleAvoider.
using AvoidFn = std::function<AvoidanceResult (const Vec3 & position, const Vec3 & desired)>;

class Mission
{
public:
  Mission(
    VehicleProfile profile, MissionConfig config, std::shared_ptr<const WorldMap> map,
    PlanFn planner, AvoidFn avoider = nullptr);

  // Plans and validates the whole delivery. Returns an empty string when the mission was
  // accepted, otherwise why it was refused. The vehicle does nothing until update().
  std::string start(const DeliveryRequest & request, const VehicleSnapshot & vehicle, double now);

  // Operator abort: go home if airborne, stand down if not.
  void cancel(const std::string & reason = "cancelled by operator");

  MissionCommand update(const VehicleSnapshot & vehicle, double now);

  MissionState state() const {return state_;}
  bool active() const;
  bool finished() const {return state_ == MissionState::Complete || state_ == MissionState::Failed;}
  const MissionResult & result() const {return result_;}
  MissionStatus status() const;
  const Path & activePath() const {return follower_.path();}
  const Path & outboundPath() const {return outbound_path_;}
  const Path & returnPath() const {return return_path_;}
  const VehicleProfile & profile() const {return profile_;}

  // State transitions and notable events since the last call, oldest first.
  std::vector<std::string> takeEvents();

private:
  enum class Leg {None, Outbound, Return};

  // Per-state handlers. Each fills `cmd` and may transition.
  void doPreflight(const VehicleSnapshot & v, double now, MissionCommand & cmd, bool relaunch);
  void doTakeoff(const VehicleSnapshot & v, double now, MissionCommand & cmd);
  void doLeg(const VehicleSnapshot & v, double now, MissionCommand & cmd);
  void doDescend(const VehicleSnapshot & v, double now, MissionCommand & cmd);
  void doRelease(const VehicleSnapshot & v, double now, MissionCommand & cmd);
  void doAscend(const VehicleSnapshot & v, double now, MissionCommand & cmd);
  void doLand(const VehicleSnapshot & v, double now, MissionCommand & cmd);

  // Safety checks that apply in every airborne state. Return true if they took over.
  bool checkExternalTakeover(const VehicleSnapshot & v, MissionCommand & cmd);
  bool checkBattery(const VehicleSnapshot & v, double now, MissionCommand & cmd);

  void transition(MissionState next, double now, const std::string & why = {});
  void startLeg(Leg leg, Path path, double now);
  void abortToReturn(const std::string & reason, const VehicleSnapshot & v, double now);
  void emergencyLand(const std::string & reason, double now);
  void fail(const std::string & reason, double now);
  void event(const std::string & text);

  bool flyingAsPlane(const VehicleSnapshot & v) const;
  bool settled(const VehicleSnapshot & v, const Vec3 & target, double now);
  bool settleBeforeLanding(
    const VehicleSnapshot & v, const Vec3 & target, double now, MissionCommand & cmd);
  Setpoint holdAt(const Vec3 & p, double yaw = std::numeric_limits<double>::quiet_NaN()) const;
  double legDeadline(double length, double now) const;
  bool airDropTrigger(const VehicleSnapshot & v, bool * on_line) const;
  std::optional<Vec3> chooseAirDropHeading(const Vec3 & target, const Vec3 & from) const;

  VehicleProfile profile_;
  MissionConfig config_;
  std::shared_ptr<const WorldMap> map_;
  PlanFn planner_;
  AvoidFn avoider_;

  MissionState state_{MissionState::Idle};
  double state_since_{0.0};
  DeliveryRequest request_;
  MissionResult result_;
  std::vector<std::string> events_;
  std::string detail_;

  // Planned geometry.
  double cruise_z_{0.0};
  Vec3 takeoff_top_;       // above the depot at cruise altitude
  Vec3 above_drop_;        // top of the descent corridor over the dropoff
  Vec3 release_point_;     // where the gripper opens (hover/winch)
  Vec3 air_drop_target_;   // over the dropoff at release altitude
  Vec3 air_drop_dir_;      // unit, horizontal
  Vec3 air_drop_entry_;    // start of the straight approach
  Vec3 air_drop_exit_;     // end of the overrun
  Path outbound_path_;
  Path return_path_;

  // Execution.
  PathFollower follower_;
  PathFollower::Progress last_progress_;
  Leg leg_{Leg::None};
  double leg_deadline_{0.0};
  Vec3 hold_xy_;               // where Takeoff climbs from
  double ready_since_{-1.0};
  double settle_since_{-1.0};
  double first_release_{-1.0};
  double last_release_{-1.0};
  int air_drop_attempt_{0};
  bool pass_missed_{false};    // this air-drop pass will not release; go around at its end
  bool was_armed_{false};
  bool aborted_{false};
  bool stand_down_{false};     // failed on the ground: keep the motors off
  bool land_via_rtl_{false};   // no planned route home; let the autopilot's RTL do it
  bool landing_committed_{false};  // stopped over the landing spot; AUTO.LAND from here
  Vec3 land_target_;           // where to stop before landing at the depot
  bool avoiding_{false};
  std::string threat_;
  std::string wanted_mode_;    // mode the mission is asking for
  std::string reached_mode_;   // ...once the vehicle has actually entered it
  std::optional<std::string> pending_cancel_;
  double last_update_{-1.0};
  std::optional<Vec3> last_position_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__MISSION_HPP_
