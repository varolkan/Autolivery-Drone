// planner_node: the city map and the global planner, outside of any mission.
//
//   autolivery/plan_path   (autolivery_interfaces/PlanPath)  plan a route without flying it
//   autolivery/map/markers (visualization_msgs/MarkerArray, latched)  the city for RViz
//   autolivery/planner/path (nav_msgs/Path, latched)  the last route planned here
//
// Useful for checking a new map or dropoff before a vehicle goes anywhere near it, and for
// comparing A* and RRT* on the same request.

#include <memory>
#include <string>

#include "autolivery_core/geometry.hpp"
#include "autolivery_drone/city_map.hpp"
#include "autolivery_drone/conversions.hpp"
#include "autolivery_drone/params.hpp"
#include "autolivery_drone/route_planner.hpp"
#include "autolivery_interfaces/srv/plan_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autolivery_drone
{

using autolivery_core::Vec3;
using visualization_msgs::msg::Marker;
using PlanPath = autolivery_interfaces::srv::PlanPath;

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("planner_node", options)
  {
    const auto map_file = resolvePackagePath(
      declare_parameter<std::string>("map_file", "maps/urban_delivery.yaml"));
    city_ = loadCityMap(map_file);
    const auto profile = declareVehicleProfile(*this);
    planner_ = std::make_unique<RoutePlanner>(
      city_.world, profile.planning_clearance, declarePlannerSettings(*this));

    const auto latched = rclcpp::QoS(1).transient_local();
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "autolivery/map/markers", latched);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("autolivery/planner/path", latched);
    service_ = create_service<PlanPath>(
      "autolivery/plan_path",
      [this](const std::shared_ptr<PlanPath::Request> req,
      std::shared_ptr<PlanPath::Response> res) {
        plan(*req, *res);
      });
    publishMarkers();
    RCLCPP_INFO(
      get_logger(), "map '%s': %zu buildings, %zu cylinders, %zu no-fly zones, grid %dx%dx%d",
      city_.name.c_str(), city_.world->buildings().size(), city_.world->cylinders().size(),
      city_.world->noFlyZones().size(), planner_->grid().sizeX(), planner_->grid().sizeY(),
      planner_->grid().sizeZ());
  }

private:
  void plan(const PlanPath::Request & req, PlanPath::Response & res)
  {
    const auto r = planner_->plan(toVec3(req.start), toVec3(req.goal), req.planner);
    res.success = r.success;
    res.message = r.message;
    res.planning_time = r.planning_time;
    res.length = autolivery_core::pathLength(r.path);
    res.path.header.stamp = now();
    res.path.header.frame_id = "world";
    for (const auto & p : r.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = res.path.header;
      pose.pose.position = toPoint(p);
      pose.pose.orientation.w = 1.0;
      res.path.poses.push_back(pose);
    }
    if (r.success) {
      path_pub_->publish(res.path);
    }
    RCLCPP_INFO(
      get_logger(), "plan %s: %s, %.0f m, %zu waypoints, %.3f s", req.planner.c_str(),
      r.message.c_str(), res.length, r.path.size(), r.planning_time);
  }

  Marker base(const std::string & ns, int id, int type)
  {
    Marker m;
    m.header.frame_id = "world";
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    auto box = [&](const std::string & ns, const autolivery_core::Box & b, float r, float g,
      float bl, float a) {
      Marker m = base(ns, id++, Marker::CUBE);
      const Vec3 c = (b.min + b.max) * 0.5;
      m.pose.position = toPoint(c);
      m.scale = toVector3(b.max - b.min);
      m.color.r = r;
      m.color.g = g;
      m.color.b = bl;
      m.color.a = a;
      arr.markers.push_back(m);
    };
    for (const auto & b : city_.world->buildings()) {
      box("buildings", b, 0.62f, 0.64f, 0.68f, 1.0f);
    }
    for (const auto & z : city_.world->noFlyZones()) {
      box("no_fly_zones", z, 0.9f, 0.1f, 0.1f, 0.25f);
    }
    for (const auto & c : city_.world->cylinders()) {
      Marker m = base("cylinders", id++, Marker::CYLINDER);
      m.pose.position = toPoint(c.base + Vec3{0.0, 0.0, c.height / 2.0});
      m.scale.x = m.scale.y = 2.0 * c.radius;
      m.scale.z = c.height;
      m.color.r = 0.2f;
      m.color.g = 0.55f;
      m.color.b = 0.2f;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }
    auto pad = [&](const std::string & ns, const std::string & name, const Vec3 & p, float r,
      float g, float b) {
      Marker m = base(ns, id++, Marker::CYLINDER);
      m.pose.position = toPoint(p + Vec3{0.0, 0.0, 0.05});
      m.scale.x = m.scale.y = 4.0;
      m.scale.z = 0.1;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
      Marker t = base(ns + "_labels", id++, Marker::TEXT_VIEW_FACING);
      t.pose.position = toPoint(p + Vec3{0.0, 0.0, 4.0});
      t.scale.z = 3.0;
      t.color.r = t.color.g = t.color.b = t.color.a = 1.0f;
      t.text = name;
      arr.markers.push_back(t);
    };
    for (const auto & [name, p] : city_.depots) {
      pad("depots", name, p, 0.1f, 0.4f, 0.9f);
    }
    for (const auto & [name, p] : city_.locations) {
      pad("locations", name, p, 1.0f, 0.55f, 0.0f);
    }
    Marker fence = base("geofence", id++, Marker::LINE_LIST);
    fence.scale.x = 0.5;
    fence.color.r = fence.color.g = 1.0f;
    fence.color.a = 0.8f;
    const Vec3 lo = city_.world->boundsMin();
    const Vec3 hi = city_.world->boundsMax();
    const Vec3 corners[4] = {{lo.x, lo.y, 0}, {hi.x, lo.y, 0}, {hi.x, hi.y, 0}, {lo.x, hi.y, 0}};
    for (int i = 0; i < 4; ++i) {
      for (double z : {lo.z, hi.z}) {
        fence.points.push_back(toPoint(corners[i] + Vec3{0, 0, z}));
        fence.points.push_back(toPoint(corners[(i + 1) % 4] + Vec3{0, 0, z}));
      }
      fence.points.push_back(toPoint(corners[i] + Vec3{0, 0, lo.z}));
      fence.points.push_back(toPoint(corners[i] + Vec3{0, 0, hi.z}));
    }
    arr.markers.push_back(fence);
    markers_pub_->publish(arr);
  }

  CityMap city_;
  std::unique_ptr<RoutePlanner> planner_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<PlanPath>::SharedPtr service_;
};

}  // namespace autolivery_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<autolivery_drone::PlannerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("planner_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
