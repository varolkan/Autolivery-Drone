// payload_node: one interface to every kind of gripper.
//
//   autolivery/payload/release  (std_srvs/Trigger)
//   autolivery/payload/grab     (std_srvs/Trigger)
//   autolivery/payload/state    (autolivery_interfaces/PayloadState, 5 Hz and on change)
//
// Hardware grippers are driven through the flight controller with MAVLink commands over
// /mavros/cmd/command (see autolivery_core/gripper.hpp for the mapping). The simulated
// gripper publishes to the Gazebo DetachableJoint topics through ros_gz_bridge.
//
// Where the parcel state comes from, in order of preference:
//   simulated gripper         the DetachableJoint's own attached/detached report (it
//                             only reports changes, so the start state is assumed)
//   payload_sensor_topic set  a limit switch / load cell (std_msgs/Bool, true = loaded)
//   neither                   inferred from successful commands, reported with sensed=false
// The mission only trusts the state for release confirmation when mission.payload_feedback
// is set; configure that only with one of the first two.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autolivery_core/gripper.hpp"
#include "autolivery_drone/node_main.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_interfaces/msg/payload_state.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace autolivery_drone
{

using autolivery_core::GripperAction;
using autolivery_core::GripperType;
using autolivery_core::MavCommand;
using PayloadMsg = autolivery_interfaces::msg::PayloadState;
using Trigger = std_srvs::srv::Trigger;

class PayloadNode : public AutoliveryNode
{
public:
  explicit PayloadNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : AutoliveryNode("payload_node", options)
  {
    gripper_ = declareGripperConfig(*this);
    // Validate the MAVLink mapping now rather than on the first release over a customer's
    // garden: gripperCommands throws on an impossible actuator index or value.
    autolivery_core::gripperCommands(gripper_, GripperAction::Release);
    autolivery_core::gripperCommands(gripper_, GripperAction::Grab);

    const auto mavros = declare_parameter<std::string>("mavros_namespace", "mavros");
    const auto detach_topic =
      declare_parameter<std::string>("sim.detach_topic", "autolivery/gripper/detach");
    const auto attach_topic =
      declare_parameter<std::string>("sim.attach_topic", "autolivery/gripper/attach");
    const auto sim_state_topic =
      declare_parameter<std::string>("sim.state_topic", "autolivery/gripper/state");
    const auto sensor_topic = declare_parameter<std::string>("payload_sensor_topic", "");
    const bool assume_loaded = declare_parameter("assume_loaded", true);
    command_timeout_ = declare_parameter("command_timeout", 3.0);

    if (gripper_.type == GripperType::Simulated) {
      // DetachableJoint only reports changes, and the parcel starts attached (the world is
      // built that way), so start from "attached" and let the first report confirm it.
      if (assume_loaded) {
        state_ = PayloadMsg::ATTACHED;
      }
      detach_pub_ = create_publisher<std_msgs::msg::Empty>(detach_topic, 10);
      attach_pub_ = create_publisher<std_msgs::msg::Empty>(attach_topic, 10);
      sim_state_sub_ = create_subscription<std_msgs::msg::String>(
        sim_state_topic, 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          // DetachableJoint reports "attached" / "detached".
          setState(
            m->data == "attached" ? PayloadMsg::ATTACHED :
            m->data == "detached" ? PayloadMsg::RELEASED : PayloadMsg::UNKNOWN, true);
        });
    } else {
      command_client_ = create_client<mavros_msgs::srv::CommandLong>(mavros + "/cmd/command");
      if (!sensor_topic.empty()) {
        sensor_sub_ = create_subscription<std_msgs::msg::Bool>(
          sensor_topic, rclcpp::SensorDataQoS(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            setState(m->data ? PayloadMsg::ATTACHED : PayloadMsg::RELEASED, true);
          });
      } else if (assume_loaded) {
        state_ = PayloadMsg::ATTACHED;
      }
    }

    state_pub_ = create_publisher<PayloadMsg>("autolivery/payload/state", 10);
    release_srv_ = create_service<Trigger>(
      "autolivery/payload/release",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>) {
        handle(GripperAction::Release, header, release_srv_);
      });
    grab_srv_ = create_service<Trigger>(
      "autolivery/payload/grab",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>) {
        handle(GripperAction::Grab, header, grab_srv_);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {publishState();});

    RCLCPP_INFO(
      get_logger(), "gripper: %s%s", autolivery_core::toString(gripper_.type).c_str(),
      gripper_.type == GripperType::Simulated ? " (Gazebo DetachableJoint)" :
      sensor_topic.empty() ? " (no payload sensor)" : (" (sensor on " + sensor_topic +
      ")").c_str());
  }

private:
  void setState(uint8_t state, bool sensed)
  {
    const bool changed = state != state_;
    state_ = state;
    sensed_ = sensed;
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "payload %s", state == PayloadMsg::ATTACHED ? "attached" :
        state == PayloadMsg::RELEASED ? "released" : "unknown");
      publishState();
    }
  }

  void publishState()
  {
    PayloadMsg msg;
    msg.header.stamp = now();
    msg.state = state_;
    msg.gripper_type = autolivery_core::toString(gripper_.type);
    msg.sensed = sensed_;
    state_pub_->publish(msg);
  }

  void respond(
    const rclcpp::Service<Trigger>::SharedPtr & srv,
    const std::shared_ptr<rmw_request_id_t> & header,
    bool success, const std::string & message)
  {
    Trigger::Response r;
    r.success = success;
    r.message = message;
    srv->send_response(*header, r);
  }

  void handle(
    GripperAction action, std::shared_ptr<rmw_request_id_t> header,
    const rclcpp::Service<Trigger>::SharedPtr & srv)
  {
    const bool release = action == GripperAction::Release;
    if (gripper_.type == GripperType::Simulated) {
      (release ? detach_pub_ : attach_pub_)->publish(std_msgs::msg::Empty());
      respond(srv, header, true, release ? "detach requested" : "attach requested");
      return;
    }
    if (busy_) {
      respond(srv, header, false, "a gripper command is already running");
      return;
    }
    if (!command_client_->service_is_ready()) {
      respond(srv, header, false, "MAVROS command service is not available");
      return;
    }
    busy_ = true;
    const auto cmds = autolivery_core::gripperCommands(gripper_, action);
    auto done = [this, header, srv, release](bool ok, const std::string & msg) {
      busy_ = false;
      deadline_.reset();
      if (ok && !sensed_) {
        // No sensor: the best we can say is that the autopilot accepted the command.
        setState(release ? PayloadMsg::RELEASED : PayloadMsg::ATTACHED, false);
      }
      respond(srv, header, ok, msg);
    };
    // One deadline for the whole sequence; a lost MAVLink ack must not wedge the gripper.
    deadline_ = create_wall_timer(
      std::chrono::duration<double>(command_timeout_ * static_cast<double>(cmds.size())),
      [this, done]() {
        if (busy_) {
          ++generation_;
          done(false, "autopilot did not acknowledge the gripper command in time");
        }
      });
    send(cmds, 0, generation_, done);
  }

  void send(
    std::vector<MavCommand> cmds, std::size_t i, unsigned generation,
    std::function<void(bool, const std::string &)> done)
  {
    if (i == cmds.size()) {
      done(true, cmds.empty() ? "nothing to do" : cmds.back().description + " acknowledged");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = cmds[i].command;
    req->param1 = cmds[i].params[0];
    req->param2 = cmds[i].params[1];
    req->param3 = cmds[i].params[2];
    req->param4 = cmds[i].params[3];
    req->param5 = cmds[i].params[4];
    req->param6 = cmds[i].params[5];
    req->param7 = cmds[i].params[6];
    RCLCPP_INFO(get_logger(), "sending %s (MAV_CMD %u)", cmds[i].description.c_str(), req->command);
    command_client_->async_send_request(
      req, [this, cmds, i, generation, done](
        rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture f) {
        if (generation != generation_) {
          return;  // timed out already
        }
        const auto r = f.get();
        if (!r->success) {
          done(
            false, cmds[i].description + " rejected by the autopilot (MAV_RESULT " +
            std::to_string(r->result) + ")");
          return;
        }
        send(cmds, i + 1, generation, done);
      });
  }

  autolivery_core::GripperConfig gripper_;
  double command_timeout_{3.0};
  uint8_t state_{PayloadMsg::UNKNOWN};
  bool sensed_{false};
  bool busy_{false};
  unsigned generation_{0};

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Publisher<PayloadMsg>::SharedPtr state_pub_;
  rclcpp::Service<Trigger>::SharedPtr release_srv_;
  rclcpp::Service<Trigger>::SharedPtr grab_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr deadline_;
};

}  // namespace autolivery_drone

int main(int argc, char ** argv)
{
  return autolivery_drone::runNode<autolivery_drone::PayloadNode>(argc, argv, "payload_node");
}
