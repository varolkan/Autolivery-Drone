// sim_traffic_node: other aircraft in the Gazebo city.
//
// Flies every route in the city map's `traffic` section back and forth at its speed, moving
// the matching `traffic_<name>` model in Gazebo with set_pose, and publishes the aircraft on
// autolivery/obstacles (source "sim_traffic") -- the same interface adsb_traffic_node fills
// from real ADS-B -- so avoidance is exercised against something that actually moves.
//
// The traffic models are static and kinematic on purpose: they are other people's aircraft
// flying their own plan, not physics objects our vehicle could push around.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>

#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>

#include "autolivery_core/geometry.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_interfaces/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolivery_gazebo
{

using autolivery_core::Vec3;

// Position and velocity on a ping-pong route at time t.
struct RouteState
{
  Vec3 position;
  Vec3 velocity;
};

RouteState routeAt(const autolivery_drone::TrafficRoute & route, double t)
{
  const auto & w = route.waypoints;
  const double length = autolivery_core::pathLength(w);
  if (length < 1e-6) {
    return {w.front(), {}};
  }
  double s = std::fmod(t * route.speed, 2.0 * length);
  const bool back = s > length;
  if (back) {
    s = 2.0 * length - s;
  }
  for (std::size_t i = 1; i < w.size(); ++i) {
    const double seg = distance(w[i - 1], w[i]);
    if (s <= seg || i + 1 == w.size()) {
      const Vec3 dir = (w[i] - w[i - 1]).normalized();
      return {w[i - 1] + dir * std::min(s, seg), dir * (back ? -route.speed : route.speed)};
    }
    s -= seg;
  }
  return {w.back(), {}};
}

class SimTrafficNode : public rclcpp::Node
{
public:
  explicit SimTrafficNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sim_traffic", options)
  {
    city_ = autolivery_drone::loadCityMap(
      autolivery_drone::resolvePackagePath(
        declare_parameter<std::string>("map_file", "maps/urban_delivery.yaml")));
    const auto world = declare_parameter<std::string>("world", city_.name);
    const double rate = declare_parameter("rate", 20.0);
    set_pose_service_ = "/world/" + world + "/set_pose";

    pub_ = create_publisher<autolivery_interfaces::msg::ObstacleArray>("autolivery/obstacles", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "flying %zu traffic route(s) in world '%s'", city_.traffic.size(),
      world.c_str());
  }

private:
  void tick()
  {
    const double t = now().seconds();
    autolivery_interfaces::msg::ObstacleArray arr;
    arr.header.stamp = now();
    arr.header.frame_id = "world";
    arr.source = "sim_traffic";
    for (const auto & route : city_.traffic) {
      const RouteState s = routeAt(route, t);

      gz::msgs::Pose req;
      req.set_name("traffic_" + route.name);
      req.mutable_position()->set_x(s.position.x);
      req.mutable_position()->set_y(s.position.y);
      req.mutable_position()->set_z(s.position.z);
      const double yaw = s.velocity.horizontalNorm() > 0.1 ?
        std::atan2(s.velocity.y, s.velocity.x) : 0.0;
      req.mutable_orientation()->set_z(std::sin(yaw / 2.0));
      req.mutable_orientation()->set_w(std::cos(yaw / 2.0));
      // Fire and forget: a dropped pose update is corrected by the next one 50 ms later,
      // and blocking here would stall the obstacle stream the avoider depends on.
      gz_.Request(set_pose_service_, req, ignore_reply_);

      autolivery_interfaces::msg::Obstacle o;
      o.id = route.name;
      o.position = autolivery_drone::toPoint(s.position);
      o.velocity = autolivery_drone::toVector3(s.velocity);
      o.radius = route.radius;
      arr.obstacles.push_back(o);
    }
    pub_->publish(arr);
  }

  autolivery_drone::CityMap city_;
  std::string set_pose_service_;
  gz::transport::Node gz_;
  std::function<void(const gz::msgs::Boolean &, const bool)> ignore_reply_ =
    [](const gz::msgs::Boolean &, const bool) {};
  rclcpp::Publisher<autolivery_interfaces::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autolivery_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<autolivery_gazebo::SimTrafficNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_traffic"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
