// mission_node: runs deliveries on a PX4 vehicle through MAVROS.
//
//   Deliver action goal -> Mission (autolivery_core) -> setpoints, mode/arming/VTOL requests,
//                                                        gripper release
//   /mavros state, pose, velocity, battery, payload state, obstacles -> VehicleSnapshot
//
// Everything that decides *what* to do lives in autolivery_core::Mission; this node only
// translates. A few things about the translation matter for safety:
//
//  * Setpoints are streamed from a timer at `control_rate` for as long as the mission wants
//    them. PX4 leaves OFFBOARD (and fails safe) if the stream drops below 2 Hz, so nothing in
//    the timer callback blocks: every service call is asynchronous and rate-limited.
//  * Vehicle state older than `pose_timeout`/`state_timeout` is treated as missing, which the
//    mission handles as "not ready" before takeoff and cannot mistake for a valid position
//    in flight.
//  * Subscriptions to MAVROS use SensorDataQoS (best effort). A best-effort subscription is
//    compatible with any publisher, whereas a reliable one silently receives nothing from
//    MAVROS topics that are published best effort -- and a mission node that never hears
//    the vehicle's position looks exactly like a vehicle that is not ready.

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autolivery_core/gripper.hpp"
#include "autolivery_core/mission.hpp"
#include "autolivery_core/velocity_obstacle.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/node_main.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_drone/route_planner.hpp"
#include "autolivery_interfaces/action/deliver.hpp"
#include "autolivery_interfaces/msg/mission_status.hpp"
#include "autolivery_interfaces/msg/obstacle_array.hpp"
#include "autolivery_interfaces/msg/payload_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace autolivery_drone
{

using autolivery_core::Mission;
using autolivery_core::MissionCommand;
using autolivery_core::MissionState;
using autolivery_core::Setpoint;
using autolivery_core::Vec3;
using autolivery_core::VehicleSnapshot;
using autolivery_core::VtolRequest;
using autolivery_core::VtolState;
using Deliver = autolivery_interfaces::action::Deliver;
using GoalHandle = rclcpp_action::ServerGoalHandle<Deliver>;
using PayloadMsg = autolivery_interfaces::msg::PayloadState;

class MissionNode : public AutoliveryNode
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : AutoliveryNode("mission_node", options)
  {
    const auto map_file = resolvePackagePath(
      declare_parameter<std::string>("map_file", "maps/urban_delivery.yaml"));
    city_ = loadCityMap(map_file);
    const auto depot_name = declare_parameter<std::string>("depot", "main");
    const auto depot = city_.depots.find(depot_name);
    if (depot == city_.depots.end()) {
      throw std::runtime_error("depot '" + depot_name + "' is not in " + map_file);
    }
    depot_ = depot->second;

    profile_ = declareVehicleProfile(*this);
    auto mission_config = declareMissionConfig(*this);
    mission_config.home = depot_;
    gripper_ = declareGripperConfig(*this);
    mission_config.release_duration =
      autolivery_core::releaseDuration(gripper_, profile_.release_altitude);
    payload_feedback_ = mission_config.payload_feedback;

    // Fixed-wings land on a runway: its approach is uploaded to PX4 as a mission landing,
    // which RTL then flies. Without a runway for this depot they fall back to PX4's circular
    // AUTO.LAND, which needs a lot of open space and is not as dependable.
    const auto runway = city_.runways.find(depot_name);
    if (profile_.vehicle_class == autolivery_core::VehicleClass::FixedWing) {
      if (runway != city_.runways.end()) {
        runway_ = runway->second;
        mission_config.fw_mission_landing = true;
      } else {
        RCLCPP_WARN(
          get_logger(), "no runway for depot '%s' in the map: the plane will land with PX4's "
          "circular AUTO.LAND", depot_name.c_str());
      }
    }
    const auto planner_settings = declarePlannerSettings(*this);
    avoidance_ = declareAvoidanceSettings(*this, profile_);

    const double rate = declare_parameter("control_rate", 20.0);
    pose_timeout_ = declare_parameter("pose_timeout", 0.5);
    state_timeout_ = declare_parameter("state_timeout", 2.0);
    const auto mavros = declare_parameter<std::string>("mavros_namespace", "mavros");
    if (!(rate >= 5.0)) {
      throw std::runtime_error("control_rate must be at least 5 Hz (PX4 needs > 2 Hz setpoints)");
    }

    route_planner_ = std::make_unique<RoutePlanner>(
      city_.world, profile_.planning_clearance, planner_settings);
    avoider_ = std::make_unique<autolivery_core::VelocityObstacleAvoider>(avoidance_.params);
    autolivery_core::AvoidFn avoid;
    if (avoidance_.enabled) {
      avoid = [this](const Vec3 & position, const Vec3 & desired) {
          return avoider_->compute(position, desired, activeObstacles(), city_.world.get());
        };
    }
    mission_ = std::make_unique<Mission>(
      profile_, mission_config, city_.world, route_planner_->asPlanFn(), avoid);

    const auto sensor = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      mavros + "/state", sensor, [this](mavros_msgs::msg::State::ConstSharedPtr m) {
        state_ = *m;
        state_time_ = nowSeconds();
      });
    extended_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      mavros + "/extended_state", sensor,
      [this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m) {
        extended_ = *m;
        extended_time_ = nowSeconds();
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      mavros + "/local_position/pose", sensor,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        pose_ = *m;
        pose_time_ = nowSeconds();
      });
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      mavros + "/local_position/velocity_local", sensor,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        velocity_ = toVec3(m->twist.linear);
        velocity_time_ = nowSeconds();
      });
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      mavros + "/battery", sensor, [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {
        battery_ = m->percentage;
        battery_time_ = nowSeconds();
      });
    payload_sub_ = create_subscription<PayloadMsg>(
      "autolivery/payload/state", 10, [this](PayloadMsg::ConstSharedPtr m) {
        payload_ = *m;
        payload_time_ = nowSeconds();
      });
    obstacle_sub_ = create_subscription<autolivery_interfaces::msg::ObstacleArray>(
      "autolivery/obstacles", 10,
      [this](autolivery_interfaces::msg::ObstacleArray::ConstSharedPtr m) {onObstacles(*m);});

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      mavros + "/setpoint_raw/local", 10);
    status_pub_ = create_publisher<autolivery_interfaces::msg::MissionStatus>(
      "autolivery/mission/status", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "autolivery/mission/path", rclcpp::QoS(1).transient_local());
    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "autolivery/mission/route", rclcpp::QoS(1).transient_local());

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(mavros + "/set_mode");
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>(mavros + "/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>(mavros + "/cmd/command");
    release_client_ = create_client<std_srvs::srv::Trigger>("autolivery/payload/release");
    mission_push_client_ =
      create_client<mavros_msgs::srv::WaypointPush>(mavros + "/mission/push");

    action_server_ = rclcpp_action::create_server<Deliver>(
      this, "autolivery/deliver",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Deliver::Goal>) {
        // Every goal is accepted so that refusals reach the client as an aborted result
        // with a reason, rather than an anonymous rejection.
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        if (goal_ && gh->get_goal_id() == goal_->get_goal_id()) {
          mission_->cancel("cancelled by operator");
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {onGoalAccepted(gh);});

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "ready: %s (%s, %s dropoff, %s gripper) at depot '%s' (%.1f, %.1f), map '%s' with %zu "
      "locations, planner %s at %.1f m, avoidance %s",
      profile_.name.c_str(), autolivery_core::toString(profile_.vehicle_class).c_str(),
      autolivery_core::toString(profile_.dropoff_method).c_str(),
      autolivery_core::toString(gripper_.type).c_str(), depot_name.c_str(), depot_.x, depot_.y,
      city_.name.c_str(), city_.locations.size(), planner_settings.algorithm.c_str(),
      planner_settings.resolution, avoidance_.enabled ? "on" : "off");
  }

  ~MissionNode() override
  {
    if (mission_ && mission_->active() && isAirborne()) {
      RCLCPP_ERROR(
        get_logger(),
        "shutting down mid-flight in state %s: setpoints stop now and PX4's offboard-loss "
        "failsafe takes over", autolivery_core::toString(mission_->state()).c_str());
    }
  }

private:
  double nowSeconds() {return now().seconds();}

  bool isAirborne() const
  {
    return extended_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND &&
           state_.armed;
  }

  VehicleSnapshot snapshot(double t) const
  {
    VehicleSnapshot s;
    s.connected = state_time_ >= 0.0 && t - state_time_ <= state_timeout_ && state_.connected;
    s.armed = state_.armed;
    s.mode = state_.mode;
    s.position_valid = pose_time_ >= 0.0 && t - pose_time_ <= pose_timeout_;
    s.position = toVec3(pose_.pose.position) + depot_;
    s.yaw = yawFromQuaternion(pose_.pose.orientation);
    s.velocity = velocity_time_ >= 0.0 && t - velocity_time_ <= pose_timeout_ ? velocity_ : Vec3{};
    if (extended_time_ >= 0.0 && t - extended_time_ <= state_timeout_) {
      s.landed = landedFromMavros(extended_.landed_state);
      s.vtol = vtolFromMavros(extended_.vtol_state);
    }
    if (battery_time_ >= 0.0 && t - battery_time_ <= 5.0 && std::isfinite(battery_) &&
      battery_ >= 0.0)
    {
      s.battery = battery_;
    }
    if (payload_feedback_ && payload_time_ >= 0.0 && t - payload_time_ <= 2.0) {
      s.payload = payload_.state == PayloadMsg::ATTACHED ? autolivery_core::PayloadState::Attached :
        payload_.state == PayloadMsg::RELEASED ? autolivery_core::PayloadState::Released :
        autolivery_core::PayloadState::Unknown;
    }
    return s;
  }

  void onObstacles(const autolivery_interfaces::msg::ObstacleArray & msg)
  {
    auto & entry = obstacles_[msg.source.empty() ? "unnamed" : msg.source];
    entry.first = nowSeconds();
    entry.second.clear();
    for (const auto & o : msg.obstacles) {
      entry.second.push_back({o.id, toVec3(o.position), toVec3(o.velocity), o.radius});
    }
  }

  std::vector<autolivery_core::DynamicObstacle> activeObstacles()
  {
    std::vector<autolivery_core::DynamicObstacle> out;
    const double t = nowSeconds();
    for (const auto & [source, entry] : obstacles_) {
      if (t - entry.first <= avoidance_.obstacle_timeout) {
        // Extrapolate to now: the avoider assumes positions are current.
        const double age = t - entry.first;
        for (auto o : entry.second) {
          o.position += o.velocity * age;
          out.push_back(o);
        }
      }
    }
    return out;
  }

  void onGoalAccepted(const std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<Deliver::Result>();
    std::string error;
    autolivery_core::DeliveryRequest request;

    if (goal_ && mission_->active()) {
      error = "a delivery is already in progress";
    } else if (!goal->location.empty()) {
      const auto it = city_.locations.find(goal->location);
      if (it == city_.locations.end()) {
        error = "unknown location '" + goal->location + "'; known:";
        for (const auto & [name, _] : city_.locations) {
          error += " " + name;
        }
      } else {
        request.dropoff = it->second;
        request.label = goal->location;
      }
    } else {
      request.dropoff = toVec3(goal->dropoff);
      if (goal->snap_to_surface) {
        request.dropoff.z = city_.world->groundHeight(request.dropoff.x, request.dropoff.y);
      }
      char label[64];
      std::snprintf(label, sizeof(label), "(%.1f, %.1f)", request.dropoff.x, request.dropoff.y);
      request.label = label;
    }

    if (error.empty() && runway_ && !landing_pattern_ready_) {
      error = "the runway landing pattern has not been accepted by the autopilot yet";
    }
    if (error.empty()) {
      const double t = nowSeconds();
      error = mission_->start(request, snapshot(t), t);
    }
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "delivery refused: %s", error.c_str());
      result->success = false;
      result->message = error;
      gh->abort(result);
      return;
    }
    goal_ = gh;
    dropoff_label_ = request.label;
    publishRoute();
    for (const auto & e : mission_->takeEvents()) {
      RCLCPP_INFO(get_logger(), "%s", e.c_str());
    }
  }

  void tick()
  {
    const double t = nowSeconds();
    const VehicleSnapshot s = snapshot(t);
    if (runway_ && !landing_pattern_ready_ && s.connected && !mission_->active()) {
      uploadLandingPattern(t);
    }
    const MissionCommand cmd = mission_->update(s, t);
    for (const auto & e : mission_->takeEvents()) {
      RCLCPP_INFO(get_logger(), "%s", e.c_str());
    }
    execute(cmd, s, t);

    const auto & path = mission_->activePath();
    if (!path.empty() && (path.size() != published_path_size_ ||
      distance(path.front(), published_path_front_) > 1e-6 ||
      distance(path.back(), published_path_back_) > 1e-6))
    {
      publishPath(path, path_pub_);
      published_path_size_ = path.size();
      published_path_front_ = path.front();
      published_path_back_ = path.back();
    }
    if (++status_counter_ % 4 == 0) {
      publishStatus(s);
    }
    if (goal_) {
      // Periodic feedback, plus one on every state change: some states (a sensed release)
      // last a single tick and clients should still see them.
      if (status_counter_ % 10 == 0 || mission_->state() != last_feedback_state_) {
        publishFeedback();
        last_feedback_state_ = mission_->state();
      }
      if (mission_->finished()) {
        finishGoal();
      }
    }
  }

  void execute(const MissionCommand & cmd, const VehicleSnapshot & s, double t)
  {
    if (cmd.setpoint.kind != Setpoint::Kind::None) {
      auto pt = toPositionTarget(cmd.setpoint, depot_);
      pt.header.stamp = now();
      pt.header.frame_id = "map";
      setpoint_pub_->publish(pt);
    }
    if (!cmd.mode.empty() && cmd.mode != s.mode) {
      requestMode(cmd.mode, t);
    }
    if (cmd.armed.has_value() && *cmd.armed != s.armed) {
      requestArming(*cmd.armed, t);
    }
    if (cmd.vtol != VtolRequest::None) {
      const bool want_fw = cmd.vtol == VtolRequest::Fw;
      const bool there = want_fw ?
        (s.vtol == VtolState::Fw || s.vtol == VtolState::TransitionToFw) :
        (s.vtol == VtolState::Mc || s.vtol == VtolState::TransitionToMc);
      if (!there && s.vtol != VtolState::Undefined) {
        requestTransition(want_fw, t);
      }
    }
    if (cmd.release) {
      requestRelease();
    }
  }

  // Service requests. Each kind has at most one call in flight and at most one per second:
  // MAVROS forwards every call to the autopilot, and a mission that asks for OFFBOARD at
  // 20 Hz while PX4 is still refusing it floods the link and the log.
  bool mayRequest(const std::string & key, double t)
  {
    auto & last = last_request_[key];
    if (in_flight_[key] || t - last < 1.0) {
      return false;
    }
    last = t;
    return true;
  }

  void requestMode(const std::string & mode, double t)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for set_mode service");
      return;
    }
    if (!mayRequest("mode", t)) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    in_flight_["mode"] = true;
    set_mode_client_->async_send_request(
      req, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f) {
        in_flight_["mode"] = false;
        if (!f.get()->mode_sent) {
          RCLCPP_WARN(get_logger(), "autopilot did not accept mode %s", mode.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "requested mode %s", mode.c_str());
        }
      });
  }

  void requestArming(bool arm, double t)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for arming service");
      return;
    }
    if (!mayRequest("arm", t)) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    in_flight_["arm"] = true;
    arming_client_->async_send_request(
      req, [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
        in_flight_["arm"] = false;
        const auto r = f.get();
        if (!r->success) {
          RCLCPP_WARN(
            get_logger(), "%s refused by the autopilot (result %u); check its preflight "
            "messages", arm ? "arming" : "disarming", r->result);
        }
      });
  }

  void requestTransition(bool to_fw, double t)
  {
    if (!command_client_->service_is_ready() || !mayRequest("vtol", t)) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = autolivery_core::kMavCmdDoVtolTransition;
    req->param1 = to_fw ? autolivery_core::kMavVtolStateFw : autolivery_core::kMavVtolStateMc;
    in_flight_["vtol"] = true;
    command_client_->async_send_request(
      req, [this, to_fw](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture f) {
        in_flight_["vtol"] = false;
        if (!f.get()->success) {
          RCLCPP_WARN(get_logger(), "VTOL transition to %s refused", to_fw ? "FW" : "MC");
        } else {
          RCLCPP_INFO(get_logger(), "VTOL transition to %s requested", to_fw ? "FW" : "MC");
        }
      });
  }

  void requestRelease()
  {
    if (!release_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "payload release service is not available");
      return;
    }
    RCLCPP_INFO(get_logger(), "releasing parcel");
    release_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        const auto r = f.get();
        if (!r->success) {
          RCLCPP_WARN(get_logger(), "release failed: %s", r->message.c_str());
        }
      });
  }

  // Uploads the runway approach as a PX4 mission: DO_LAND_START, loiter down to the approach
  // altitude, straight-in NAV_LAND. With RTL_TYPE 1 (PX4's fixed-wing default) RTL flies it.
  void uploadLandingPattern(double t)
  {
    if (!mission_push_client_->service_is_ready() || in_flight_["mission"] ||
      t - last_request_["mission"] < 5.0)
    {
      return;
    }
    last_request_["mission"] = t;
    using WP = mavros_msgs::msg::Waypoint;
    const auto item = [this](uint16_t command, const Vec3 & where) {
        WP wp;
        wp.frame = WP::FRAME_GLOBAL_REL_ALT;  // altitude above home, i.e. the depot
        wp.command = command;
        wp.autocontinue = true;
        const Vec3 geo = worldToGeodetic(city_.origin, where);
        wp.x_lat = geo.x;
        wp.y_long = geo.y;
        wp.z_alt = where.z - depot_.z;
        return wp;
      };
    constexpr uint16_t kDoLandStart = 189;
    constexpr uint16_t kNavLoiterToAlt = 31;
    constexpr uint16_t kNavLand = 21;
    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    WP start = item(kDoLandStart, {runway_->approach.x, runway_->approach.y, depot_.z});
    // PX4 only accepts DO_* items in the MISSION frame; a global frame is "unsupported".
    start.frame = WP::FRAME_MISSION;
    start.is_current = true;
    WP loiter = item(kNavLoiterToAlt, runway_->approach);
    loiter.param2 = static_cast<float>(runway_->loiter_radius);
    WP land = item(kNavLand, runway_->touchdown);
    land.param4 = std::numeric_limits<float>::quiet_NaN();  // keep the approach heading
    req->waypoints = {start, loiter, land};
    in_flight_["mission"] = true;
    mission_push_client_->async_send_request(
      req, [this](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture f) {
        in_flight_["mission"] = false;
        const auto r = f.get();
        if (r->success && r->wp_transfered == 3) {
          landing_pattern_ready_ = true;
          RCLCPP_INFO(
            get_logger(), "runway '%s' landing pattern accepted by the autopilot (%.1f deg glide "
            "slope, landing heading %.0f deg)", runway_->name.c_str(), runway_->glide_slope_deg,
            std::fmod(
              std::atan2(runway_->direction.x, runway_->direction.y) * 180.0 /
              autolivery_core::kPi + 360.0, 360.0));
        } else {
          RCLCPP_WARN(
            get_logger(), "autopilot rejected the landing pattern (%u of 3 items); retrying",
            r->wp_transfered);
        }
      });
  }

  void publishPath(
    const autolivery_core::Path & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    for (const auto & p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = toPoint(p);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    pub->publish(msg);
  }

  void publishRoute()
  {
    autolivery_core::Path route = mission_->outboundPath();
    route.insert(route.end(), mission_->returnPath().begin(), mission_->returnPath().end());
    publishPath(route, route_pub_);
  }

  void publishStatus(const VehicleSnapshot & s)
  {
    const auto st = mission_->status();
    autolivery_interfaces::msg::MissionStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.state = autolivery_core::toString(st.state);
    msg.vehicle = profile_.name;
    msg.dropoff = mission_->active() || mission_->finished() ? dropoff_label_ : "";
    msg.leg = st.leg;
    msg.leg_remaining = st.leg_remaining;
    msg.waypoint = static_cast<uint32_t>(st.waypoint);
    msg.waypoint_count = static_cast<uint32_t>(st.waypoint_count);
    msg.avoiding = st.avoiding;
    msg.threat = st.threat;
    msg.detail = st.detail;
    msg.connected = s.connected;
    msg.armed = s.armed;
    msg.flight_mode = s.mode;
    msg.battery = s.battery;
    msg.position = toPoint(s.position);
    msg.payload = s.payload == autolivery_core::PayloadState::Attached ? "attached" :
      s.payload == autolivery_core::PayloadState::Released ? "released" : "unknown";
    status_pub_->publish(msg);
  }

  void publishFeedback()
  {
    const auto st = mission_->status();
    auto fb = std::make_shared<Deliver::Feedback>();
    fb->state = autolivery_core::toString(st.state);
    fb->detail = st.detail;
    fb->leg = st.leg;
    fb->leg_remaining = st.leg_remaining;
    fb->waypoint = static_cast<uint32_t>(st.waypoint);
    fb->waypoint_count = static_cast<uint32_t>(st.waypoint_count);
    fb->avoiding = st.avoiding;
    goal_->publish_feedback(fb);
  }

  void finishGoal()
  {
    const auto & r = mission_->result();
    auto result = std::make_shared<Deliver::Result>();
    result->success = r.success;
    result->delivered = r.delivered;
    result->message = r.message;
    result->flight_time = r.flight_time;
    result->distance_flown = r.distance_flown;
    if (goal_->is_canceling()) {
      goal_->canceled(result);
    } else if (r.success) {
      goal_->succeed(result);
    } else {
      goal_->abort(result);
    }
    RCLCPP_INFO(
      get_logger(), "delivery %s: %s (%.0f s, %.0f m)", r.success ? "succeeded" : "failed",
      r.message.c_str(), r.flight_time, r.distance_flown);
    goal_.reset();
  }

  // Configuration.
  CityMap city_;
  Vec3 depot_;
  autolivery_core::VehicleProfile profile_;
  autolivery_core::GripperConfig gripper_;
  AvoidanceSettings avoidance_;
  bool payload_feedback_{false};
  std::optional<Runway> runway_;
  bool landing_pattern_ready_{false};
  double pose_timeout_{0.5};
  double state_timeout_{2.0};

  std::unique_ptr<RoutePlanner> route_planner_;
  std::unique_ptr<autolivery_core::VelocityObstacleAvoider> avoider_;
  std::unique_ptr<Mission> mission_;

  // Latest vehicle data and when it arrived (node clock, seconds; -1 = never).
  mavros_msgs::msg::State state_;
  double state_time_{-1.0};
  mavros_msgs::msg::ExtendedState extended_;
  double extended_time_{-1.0};
  geometry_msgs::msg::PoseStamped pose_;
  double pose_time_{-1.0};
  Vec3 velocity_;
  double velocity_time_{-1.0};
  double battery_{-1.0};
  double battery_time_{-1.0};
  PayloadMsg payload_;
  double payload_time_{-1.0};
  std::map<std::string,
    std::pair<double, std::vector<autolivery_core::DynamicObstacle>>> obstacles_;

  std::map<std::string, double> last_request_;
  std::map<std::string, bool> in_flight_;
  std::shared_ptr<GoalHandle> goal_;
  std::string dropoff_label_;
  std::size_t published_path_size_{0};
  Vec3 published_path_front_;
  Vec3 published_path_back_;
  unsigned status_counter_{0};
  MissionState last_feedback_state_{MissionState::Idle};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<PayloadMsg>::SharedPtr payload_sub_;
  rclcpp::Subscription<autolivery_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<autolivery_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr mission_push_client_;
  rclcpp_action::Server<Deliver>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autolivery_drone

int main(int argc, char ** argv)
{
  return autolivery_drone::runNode<autolivery_drone::MissionNode>(argc, argv, "mission_node");
}
