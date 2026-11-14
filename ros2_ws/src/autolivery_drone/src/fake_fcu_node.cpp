// fake_fcu_node: the MAVROS API backed by a kinematic vehicle, for dry runs and CI.
//
// Publishes and serves the same topics and services mission_node and payload_node use on a
// real vehicle -- mavros/state, extended_state, local_position/*, battery, set_mode,
// cmd/arming, cmd/command, setpoint_raw/local -- and emulates the Gazebo gripper topics, so
// the whole ROS side of a delivery can run without PX4 or Gazebo. The vehicle model is
// autolivery_core::KinematicVehicle, which enforces the PX4 handshakes that matter (setpoint
// stream before OFFBOARD, arming only in OFFBOARD/AUTO.TAKEOFF, auto-disarm after AUTO.LAND).
//
// It can also be the weather: `wind_speed`, `wind_direction` and `wind_gust` blow on the
// vehicle, and with `report_weather` the node reports them on autolivery/weather the way a
// station at the depot would (with `precipitation` and `temperature`, if given).
//
// This is a test tool. Nothing here should ever be launched next to a real autopilot.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "autolivery_core/gripper.hpp"
#include "autolivery_core/kinematic_vehicle.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/node_main.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_interfaces/msg/weather.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace autolivery_drone
{

using autolivery_core::Vec3;

class FakeFcuNode : public AutoliveryNode
{
public:
  explicit FakeFcuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : AutoliveryNode("fake_fcu", options)
  {
    city_ = loadCityMap(
      resolvePackagePath(
        declare_parameter<std::string>("map_file", "maps/urban_delivery.yaml")));
    const auto depot = declare_parameter<std::string>("depot", "main");
    if (!city_.depots.count(depot)) {
      throw std::runtime_error("depot '" + depot + "' is not in the map");
    }
    depot_ = city_.depots.at(depot);
    const auto profile = declareVehicleProfile(*this);
    vehicle_ = std::make_unique<autolivery_core::KinematicVehicle>(profile, depot_, city_.world);
    vehicle_->record_trajectory = false;
    vehicle_->drain_per_s = declare_parameter("battery_drain_per_s", 0.0005);
    vehicle_->gripper_stuck = declare_parameter("gripper_stuck", false);
    gripper_rejections_ = static_cast<int>(declare_parameter("gripper_rejections", 0));
    vehicle_->runway_heading = declare_parameter("runway_heading", 0.0);
    boot_delay_ = declare_parameter("boot_delay", 2.0);
    constexpr double kNotReported = std::numeric_limits<double>::quiet_NaN();
    weather_.wind_speed = static_cast<float>(declare_parameter("wind_speed", 0.0));
    weather_.wind_direction = static_cast<float>(declare_parameter("wind_direction", 0.0));
    weather_.wind_gust = static_cast<float>(
      std::max(declare_parameter("wind_gust", 0.0), static_cast<double>(weather_.wind_speed)));
    weather_.precipitation = static_cast<float>(declare_parameter("precipitation", kNotReported));
    weather_.temperature = static_cast<float>(declare_parameter("temperature", kNotReported));
    vehicle_->wind = windFromMeteorological(weather_.wind_speed, weather_.wind_direction);
    vehicle_->gust = weather_.wind_gust - weather_.wind_speed;
    if (declare_parameter("report_weather", false)) {
      weather_pub_ = create_publisher<autolivery_interfaces::msg::Weather>(
        "autolivery/weather", 10);
    }
    const double rate = declare_parameter("rate", 50.0);
    const auto mavros = declare_parameter<std::string>("mavros_namespace", "mavros");

    const auto sensor = rclcpp::SensorDataQoS();
    state_pub_ = create_publisher<mavros_msgs::msg::State>(mavros + "/state", sensor);
    extended_pub_ = create_publisher<mavros_msgs::msg::ExtendedState>(
      mavros + "/extended_state", sensor);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      mavros + "/local_position/pose", sensor);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      mavros + "/local_position/velocity_local", sensor);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(mavros + "/battery", sensor);
    world_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "autolivery/fake_fcu/pose", 10);

    setpoint_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      mavros + "/setpoint_raw/local", 10,
      [this](mavros_msgs::msg::PositionTarget::ConstSharedPtr m) {
        vehicle_->setSetpoint(fromPositionTarget(*m, depot_), now().seconds());
      });
    set_mode_srv_ = create_service<mavros_msgs::srv::SetMode>(
      mavros + "/set_mode",
      [this](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
      std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {
        // Like MAVROS: mode_sent means the request went out; whether the autopilot took it
        // shows up in the next state message.
        vehicle_->requestMode(req->custom_mode, now().seconds());
        res->mode_sent = true;
      });
    arming_srv_ = create_service<mavros_msgs::srv::CommandBool>(
      mavros + "/cmd/arming",
      [this](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
      std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res) {
        res->success = vehicle_->requestArm(req->value);
        res->result = res->success ? 0 : 4;  // MAV_RESULT_ACCEPTED / MAV_RESULT_FAILED
      });
    command_srv_ = create_service<mavros_msgs::srv::CommandLong>(
      mavros + "/cmd/command",
      [this](const std::shared_ptr<mavros_msgs::srv::CommandLong::Request> req,
      std::shared_ptr<mavros_msgs::srv::CommandLong::Response> res) {onCommand(*req, *res);});

    mission_push_srv_ = create_service<mavros_msgs::srv::WaypointPush>(
      mavros + "/mission/push",
      [this](const std::shared_ptr<mavros_msgs::srv::WaypointPush::Request> req,
      std::shared_ptr<mavros_msgs::srv::WaypointPush::Response> res) {
        // Only the landing pattern matters here: with one loaded, fixed-wing RTL lands.
        for (const auto & wp : req->waypoints) {
          if (wp.command == 21) {  // MAV_CMD_NAV_LAND
            vehicle_->fixed_wing_rtl_lands = true;
          }
        }
        res->success = true;
        res->wp_transfered = static_cast<uint32_t>(req->waypoints.size());
      });

    // Stand-in for the Gazebo DetachableJoint the simulated gripper talks to.
    detach_sub_ = create_subscription<std_msgs::msg::Empty>(
      "autolivery/gripper/detach", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {vehicle_->releasePayload();});
    attach_sub_ = create_subscription<std_msgs::msg::Empty>(
      "autolivery/gripper/attach", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {vehicle_->attachPayload();});
    gripper_state_pub_ = create_publisher<std_msgs::msg::String>("autolivery/gripper/state", 10);

    start_ = now().seconds();
    last_step_ = start_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {step();});
    RCLCPP_WARN(
      get_logger(), "FAKE flight controller (%s) at depot '%s'; not for use with real vehicles",
      profile.name.c_str(), depot.c_str());
  }

private:
  void onCommand(
    const mavros_msgs::srv::CommandLong::Request & req,
    mavros_msgs::srv::CommandLong::Response & res)
  {
    res.success = true;
    res.result = 0;
    const bool gripper_command = req.command == autolivery_core::kMavCmdDoGripper ||
      req.command == autolivery_core::kMavCmdDoWinch ||
      req.command == autolivery_core::kMavCmdDoSetActuator;
    if (gripper_command && gripper_rejections_ > 0) {
      // An autopilot that refuses the command (payload deliverer not enabled, output not
      // mapped): the first `gripper_rejections` are answered MAV_RESULT_DENIED.
      --gripper_rejections_;
      res.success = false;
      res.result = 2;
      return;
    }
    switch (req.command) {
      case autolivery_core::kMavCmdDoVtolTransition:
        vehicle_->requestVtol(
          req.param1 == autolivery_core::kMavVtolStateFw ?
          autolivery_core::VtolRequest::Fw : autolivery_core::VtolRequest::Mc, now().seconds());
        break;
      case autolivery_core::kMavCmdDoGripper:
        if (req.param2 == 0.0f) {
          vehicle_->releasePayload();
        } else {
          vehicle_->attachPayload();
        }
        break;
      case autolivery_core::kMavCmdDoWinch:
      case autolivery_core::kMavCmdDoSetActuator:
        // Release semantics depend on the configured values; the dry run treats any of
        // these as "the parcel came off" unless it is a retract/lock.
        if (!(req.command == autolivery_core::kMavCmdDoWinch && (req.param2 == 3.0f ||
          req.param2 == 6.0f)))
        {
          vehicle_->releasePayload();
        }
        break;
      default:
        res.success = false;
        res.result = 3;  // MAV_RESULT_UNSUPPORTED
    }
  }

  void step()
  {
    const double t = now().seconds();
    const double dt = std::clamp(t - last_step_, 0.0, 0.1);
    last_step_ = t;
    // A real FCU takes a moment to boot and get a position fix.
    vehicle_->connected = t - start_ >= boot_delay_;
    vehicle_->step(t, dt);

    const auto s = vehicle_->snapshot();
    const auto stamp = now();
    mavros_msgs::msg::State state;
    state.header.stamp = stamp;
    state.connected = s.connected;
    state.armed = s.armed;
    state.guided = s.mode == "OFFBOARD";
    state.mode = s.mode;
    state_pub_->publish(state);

    mavros_msgs::msg::ExtendedState ext;
    ext.header.stamp = stamp;
    ext.landed_state = landedToMavros(s.landed);
    ext.vtol_state = vtolToMavros(s.vtol);
    extended_pub_->publish(ext);

    if (s.connected) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "map";
      pose.pose.position = toPoint(s.position - depot_);
      pose.pose.orientation = quaternionFromYaw(s.yaw);
      pose_pub_->publish(pose);
      pose.header.frame_id = "world";
      pose.pose.position = toPoint(s.position);
      world_pose_pub_->publish(pose);

      geometry_msgs::msg::TwistStamped twist;
      twist.header = pose.header;
      twist.header.frame_id = "map";
      twist.twist.linear = toVector3(s.velocity);
      velocity_pub_->publish(twist);
    }

    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamp;
    battery.percentage = static_cast<float>(s.battery);
    battery.voltage = static_cast<float>(14.0 + 2.8 * s.battery);
    battery.present = true;
    battery_pub_->publish(battery);

    if (++gripper_counter_ % 10 == 0) {
      std_msgs::msg::String g;
      g.data = vehicle_->payload() == autolivery_core::PayloadState::Attached ? "attached" :
        "detached";
      gripper_state_pub_->publish(g);
      if (weather_pub_) {
        weather_.header.stamp = stamp;
        weather_pub_->publish(weather_);
      }
    }
  }

  CityMap city_;
  Vec3 depot_;
  std::unique_ptr<autolivery_core::KinematicVehicle> vehicle_;
  double boot_delay_{2.0};
  int gripper_rejections_{0};
  autolivery_interfaces::msg::Weather weather_;
  double start_{0.0};
  double last_step_{0.0};
  unsigned gripper_counter_{0};

  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr extended_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_state_pub_;
  rclcpp::Publisher<autolivery_interfaces::msg::Weather>::SharedPtr weather_pub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr detach_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr attach_sub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv_;
  rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_srv_;
  rclcpp::Service<mavros_msgs::srv::WaypointPush>::SharedPtr mission_push_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autolivery_drone

int main(int argc, char ** argv)
{
  return autolivery_drone::runNode<autolivery_drone::FakeFcuNode>(argc, argv, "fake_fcu");
}
