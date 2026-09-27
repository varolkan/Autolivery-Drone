// Reactive avoidance of moving obstacles (other aircraft, cranes, birds the lidar tracks).
//
// The global planner keeps the route clear of everything in the prior map. What it cannot
// know about is traffic, so this sits between the path follower and the autopilot and bends
// the commanded velocity around anything predicted to come too close within the time horizon.
//
// It is a sampled velocity-obstacle method: try the desired velocity first, then a fan of
// alternatives (turn, slow down, climb, descend, stop), and pick the closest one to what the
// path follower asked for that keeps the required separation from every obstacle -- assuming
// they hold their current velocity -- and does not fly into the static map. Potential fields
// were the other obvious choice; they get stuck in local minima between buildings and do
// not use obstacle velocity at all, which is the whole point here.

#ifndef AUTOLIVERY_CORE__VELOCITY_OBSTACLE_HPP_
#define AUTOLIVERY_CORE__VELOCITY_OBSTACLE_HPP_

#include <string>
#include <vector>

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

struct DynamicObstacle
{
  std::string id;
  Vec3 position;
  Vec3 velocity;
  double radius{1.0};
};

struct AvoidanceParams
{
  double time_horizon{8.0};         // s, conflicts further out than this are ignored
  double vehicle_radius{0.6};
  double safety_margin{3.0};        // separation kept on top of both radii
  double max_horizontal_speed{12.0};
  double max_climb_rate{3.0};
  double max_descent_rate{1.5};
  double min_speed{0.0};            // fixed-wing: candidates slower than this are not flyable
  int heading_samples{12};          // per side of the desired heading
  double max_heading_change{2.4};   // rad, widest turn considered (~135 deg)
  double static_clearance{1.5};     // kept from the prior map by avoidance manoeuvres
  double static_horizon{3.0};       // s, how far ahead manoeuvres are checked against the map
};

struct AvoidanceResult
{
  Vec3 velocity;
  bool modified{false};      // velocity differs from the request
  bool conflict_free{true};  // false: nothing met the separation; this is the least-bad option
  std::string threat;        // id of the obstacle that forced the change
  double min_separation{0.0};  // predicted closest approach margin for the chosen velocity (m)
};

class VelocityObstacleAvoider
{
public:
  explicit VelocityObstacleAvoider(AvoidanceParams params = {});

  AvoidanceResult compute(
    const Vec3 & position, const Vec3 & desired,
    const std::vector<DynamicObstacle> & obstacles, const WorldMap * map) const;

  const AvoidanceParams & params() const {return params_;}

private:
  // Smallest (closest approach - required separation) over all obstacles and the horizon.
  double separationMargin(
    const Vec3 & position, const Vec3 & velocity,
    const std::vector<DynamicObstacle> & obstacles, std::string * worst) const;

  bool staticallyClear(const Vec3 & position, const Vec3 & velocity, const WorldMap * map) const;

  AvoidanceParams params_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__VELOCITY_OBSTACLE_HPP_
