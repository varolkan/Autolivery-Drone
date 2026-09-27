// adsb_traffic_node: other aircraft from ADS-B (via MAVROS) as obstacles for avoidance.
//
//   mavros/adsb/vehicle (mavros_msgs/ADSBVehicle)  ->  autolivery/obstacles (source "adsb")
//
// Reports are converted to the world frame around the city map's geodetic origin and
// republished at `rate` with their age accounted for by the mission node. Aircraft not heard
// from for `timeout` seconds are dropped. Reports without valid coordinates or altitude are
// ignored rather than guessed at.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_interfaces/msg/obstacle_array.hpp"
#include "mavros_msgs/msg/adsb_vehicle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolivery_drone
{

using ADSB = mavros_msgs::msg::ADSBVehicle;

class AdsbTrafficNode : public rclcpp::Node
{
public:
  explicit AdsbTrafficNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("adsb_traffic_node", options)
  {
    city_ = loadCityMap(
      resolvePackagePath(
        declare_parameter<std::string>("map_file", "maps/urban_delivery.yaml")));
    const auto mavros = declare_parameter<std::string>("mavros_namespace", "mavros");
    timeout_ = declare_parameter("timeout", 3.0);
    uav_radius_ = declare_parameter("uav_radius", 3.0);
    aircraft_radius_ = declare_parameter("aircraft_radius", 30.0);
    const double rate = declare_parameter("rate", 5.0);

    sub_ = create_subscription<ADSB>(
      mavros + "/adsb/vehicle", rclcpp::SensorDataQoS(),
      [this](ADSB::ConstSharedPtr m) {onReport(*m);});
    pub_ = create_publisher<autolivery_interfaces::msg::ObstacleArray>("autolivery/obstacles", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {publish();});
  }

private:
  void onReport(const ADSB & m)
  {
    if (!(m.flags & ADSB::FLAG_VALID_COORDS) || !(m.flags & ADSB::FLAG_VALID_ALTITUDE)) {
      return;
    }
    autolivery_interfaces::msg::Obstacle o;
    o.id = m.callsign.empty() ? std::to_string(m.icao_address) : m.callsign;
    o.position = toPoint(geodeticToWorld(city_.origin, m.latitude, m.longitude, m.altitude));
    if (m.flags & ADSB::FLAG_VALID_HEADING && m.flags & ADSB::FLAG_VALID_VELOCITY) {
      // ADS-B heading is degrees clockwise from north; world is ENU.
      const double h = m.heading * autolivery_core::kPi / 180.0;
      o.velocity.x = m.hor_velocity * std::sin(h);
      o.velocity.y = m.hor_velocity * std::cos(h);
    }
    if (m.flags & ADSB::FLAG_VERTICAL_VELOCITY_VALID) {
      o.velocity.z = m.ver_velocity;
    }
    o.radius = m.emitter_type == ADSB::EMITTER_UAV ? uav_radius_ : aircraft_radius_;
    traffic_[m.icao_address] = {now().seconds(), o};
  }

  void publish()
  {
    const double t = now().seconds();
    autolivery_interfaces::msg::ObstacleArray arr;
    arr.header.stamp = now();
    arr.header.frame_id = "world";
    arr.source = "adsb";
    for (auto it = traffic_.begin(); it != traffic_.end(); ) {
      if (t - it->second.first > timeout_) {
        it = traffic_.erase(it);
        continue;
      }
      auto o = it->second.second;
      const double age = t - it->second.first;
      o.position.x += o.velocity.x * age;
      o.position.y += o.velocity.y * age;
      o.position.z += o.velocity.z * age;
      arr.obstacles.push_back(o);
      ++it;
    }
    pub_->publish(arr);
  }

  CityMap city_;
  double timeout_{3.0};
  double uav_radius_{3.0};
  double aircraft_radius_{30.0};
  std::map<uint32_t, std::pair<double, autolivery_interfaces::msg::Obstacle>> traffic_;
  rclcpp::Subscription<ADSB>::SharedPtr sub_;
  rclcpp::Publisher<autolivery_interfaces::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autolivery_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<autolivery_drone::AdsbTrafficNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("adsb_traffic_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
