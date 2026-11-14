// sim_weather_node: the weather in the Gazebo city.
//
// Blows the wind it is given on the simulated world -- a mean wind with gusts along it --
// and reports it on autolivery/weather the way a station at the depot would, so the flight
// stack's weather limits are held against the same wind the physics is using.
//
// Gazebo has one wind for the whole world. A multirotor feels it as drag on the airframe
// (the WindEffects system; sim_models.py turns it on for airframes without aerodynamics of
// their own), planes and VTOLs through their lift and drag surfaces, which fly in the moving
// air. Rain and temperature have no physics here: they are only reported.

#include <gz/msgs/wind.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gz/transport/Node.hh>

#include "autolivery_core/geometry.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/node_main.hpp"
#include "autolivery_interfaces/msg/weather.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolivery_gazebo
{

using autolivery_core::kPi;
using autolivery_core::Vec3;

class SimWeatherNode : public autolivery_drone::AutoliveryNode
{
public:
  explicit SimWeatherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : autolivery_drone::AutoliveryNode("sim_weather", options)
  {
    constexpr double kNotReported = std::numeric_limits<double>::quiet_NaN();
    const auto world = declare_parameter<std::string>("world", "urban_delivery");
    speed_ = declare_parameter("wind_speed", 0.0);
    direction_ = declare_parameter("wind_direction", 0.0);
    gust_ = std::max(declare_parameter("wind_gust", 0.0), speed_);
    report_.wind_speed = static_cast<float>(speed_);
    report_.wind_direction = static_cast<float>(direction_);
    report_.wind_gust = static_cast<float>(gust_);
    report_.precipitation = static_cast<float>(declare_parameter("precipitation", kNotReported));
    report_.temperature = static_cast<float>(declare_parameter("temperature", kNotReported));
    if (!(speed_ >= 0.0)) {
      throw std::runtime_error("wind_speed must not be negative");
    }

    wind_pub_ = gz_.Advertise<gz::msgs::Wind>("/world/" + world + "/wind");
    if (declare_parameter("report", true)) {
      report_pub_ = create_publisher<autolivery_interfaces::msg::Weather>(
        "autolivery/weather", 10);
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100), [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "wind in world '%s': %.1f m/s from %.0f deg, gusts to %.1f m/s%s",
      world.c_str(), speed_, direction_, gust_, report_pub_ ? ", reported on autolivery/weather" :
      ", not reported");
  }

private:
  void tick()
  {
    // The same repeatable gusts as the kinematic vehicle: two slow sines along the wind,
    // swinging it either side of the mean (never backwards) and peaking at the gust speed.
    const double t = now().seconds();
    const double swing =
      0.6 * std::sin(2.0 * kPi * t / 9.0) + 0.4 * std::sin(2.0 * kPi * t / 3.7);
    const double speed = std::max(0.0, speed_ + (gust_ - speed_) * swing);
    const Vec3 air = autolivery_drone::windFromMeteorological(speed, direction_);

    gz::msgs::Wind wind;
    wind.mutable_linear_velocity()->set_x(air.x);
    wind.mutable_linear_velocity()->set_y(air.y);
    wind.mutable_linear_velocity()->set_z(0.0);
    wind.set_enable_wind(true);
    wind_pub_.Publish(wind);

    if (report_pub_ && ++ticks_ % 10 == 0) {
      report_.header.stamp = now();
      report_pub_->publish(report_);
    }
  }

  double speed_{0.0};
  double direction_{0.0};
  double gust_{0.0};
  unsigned ticks_{0};
  autolivery_interfaces::msg::Weather report_;
  gz::transport::Node gz_;
  gz::transport::Node::Publisher wind_pub_;
  rclcpp::Publisher<autolivery_interfaces::msg::Weather>::SharedPtr report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autolivery_gazebo

int main(int argc, char ** argv)
{
  return autolivery_drone::runNode<autolivery_gazebo::SimWeatherNode>(argc, argv, "sim_weather");
}
