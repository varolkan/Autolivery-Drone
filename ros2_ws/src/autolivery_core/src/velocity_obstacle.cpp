#include "autolivery_core/velocity_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autolivery_core
{

VelocityObstacleAvoider::VelocityObstacleAvoider(AvoidanceParams params)
: params_(params)
{
  if (!(params_.time_horizon > 0.0) || params_.heading_samples < 1) {
    throw std::invalid_argument("avoidance needs a positive horizon and at least one sample");
  }
}

double VelocityObstacleAvoider::separationMargin(
  const Vec3 & position, const Vec3 & velocity,
  const std::vector<DynamicObstacle> & obstacles, std::string * worst) const
{
  double margin = std::numeric_limits<double>::infinity();
  for (const auto & o : obstacles) {
    // Relative motion of the obstacle as seen from the vehicle; closest approach of a
    // constant-velocity pair happens at t* = -p.v / |v|^2, clamped to the horizon.
    const Vec3 p = o.position - position;
    const Vec3 v = o.velocity - velocity;
    const double vv = v.dot(v);
    double t = 0.0;
    if (vv > 1e-9) {
      t = std::clamp(-p.dot(v) / vv, 0.0, params_.time_horizon);
    }
    const double closest = (p + v * t).norm();
    const double required = o.radius + params_.vehicle_radius + params_.safety_margin;
    const double m = closest - required;
    if (m < margin) {
      margin = m;
      if (worst != nullptr) {
        *worst = o.id;
      }
    }
  }
  return margin;
}

bool VelocityObstacleAvoider::staticallyClear(
  const Vec3 & position, const Vec3 & velocity, const WorldMap * map) const
{
  if (map == nullptr || velocity.norm() < 1e-6) {
    return true;
  }
  const Vec3 end = position + velocity * params_.static_horizon;
  // The vehicle may already be inside the static margin (it is following a planned path that
  // hugs a corner at planning clearance); what matters is that the manoeuvre does not take it
  // into the map. So check against the margin only if the start point is outside it.
  const double c = map->isBlocked(position, params_.static_clearance) ? 0.0 :
    params_.static_clearance;
  return map->segmentClear(position, end, c);
}

AvoidanceResult VelocityObstacleAvoider::compute(
  const Vec3 & position, const Vec3 & desired,
  const std::vector<DynamicObstacle> & obstacles, const WorldMap * map) const
{
  AvoidanceResult result;
  result.velocity = desired;

  std::string threat;
  const double desired_margin = separationMargin(position, desired, obstacles, &threat);
  result.min_separation = desired_margin;
  if (desired_margin >= 0.0) {
    return result;
  }

  struct Candidate
  {
    Vec3 v;
    double cost;
    double margin;
    double escape;  // separation margin one second from now
  };
  std::vector<Candidate> candidates;

  const double desired_h = desired.horizontalNorm();
  // With no horizontal intent (hovering, vertical legs) there is no heading to fan around, so
  // escape in any direction at a modest speed.
  const double base_speed = desired_h > 0.5 ?
    desired_h : std::min(3.0, params_.max_horizontal_speed);
  const double base_heading = desired_h > 0.5 ? std::atan2(desired.y, desired.x) : 0.0;
  const double max_turn = desired_h > 0.5 ? params_.max_heading_change : kPi;
  const int n = params_.heading_samples;
  const double vz_step = std::max(1.0, 0.66 * std::min(params_.max_climb_rate, 3.0));
  const double vertical_options[] = {desired.z, desired.z + vz_step, desired.z - vz_step};
  const double speed_factors[] = {1.0, 0.6, 0.3};

  auto consider = [&](const Vec3 & raw) {
      const Vec3 v = clampVelocity(
      raw, params_.max_horizontal_speed, params_.max_climb_rate, params_.max_descent_rate);
      if (params_.min_speed > 0.0 && v.horizontalNorm() < params_.min_speed) {
        return;
      }
      if (!staticallyClear(position, v, map)) {
        return;
      }
      const double margin = separationMargin(position, v, obstacles, nullptr);
      double escape = std::numeric_limits<double>::infinity();
      for (const auto & o : obstacles) {
        const Vec3 later = o.position + o.velocity * 1.0 - (position + v * 1.0);
        escape = std::min(
        escape, later.norm() - (o.radius + params_.vehicle_radius + params_.safety_margin));
      }
      const Vec3 dv = v - desired;
    // Normalised deviation from the request; vertical deviation costs a bit more because
    // leaving the planned altitude band takes the vehicle out of deconflicted airspace.
      const double scale = std::max(1.0, params_.max_horizontal_speed);
      const double cost = (dv.x * dv.x + dv.y * dv.y + 1.5 * dv.z * dv.z) / (scale * scale);
      candidates.push_back({v, cost, margin, escape});
    };

  for (int i = -n; i <= n; ++i) {
    const double heading = base_heading + max_turn * static_cast<double>(i) / n;
    for (double f : speed_factors) {
      const double s = base_speed * f;
      for (double vz : vertical_options) {
        consider({std::cos(heading) * s, std::sin(heading) * s, vz});
      }
    }
  }
  // Pure vertical escapes and a full stop, for vehicles that can hover.
  if (params_.min_speed <= 0.0) {
    consider({0.0, 0.0, 0.0});
    consider({0.0, 0.0, vz_step});
    consider({0.0, 0.0, -vz_step});
  }

  result.modified = true;
  result.threat = threat;
  if (candidates.empty()) {
    // Everything flies into the map. Stopping is the only honest answer for a multirotor; a
    // fixed-wing keeps its heading and the mission layer escalates.
    result.velocity = params_.min_speed > 0.0 ? desired : Vec3{};
    result.conflict_free = false;
    return result;
  }

  const Candidate * best = nullptr;
  for (const auto & c : candidates) {
    if (c.margin >= 0.0 && (best == nullptr || c.cost < best->cost)) {
      best = &c;
    }
  }
  if (best == nullptr) {
    // No candidate keeps full separation: take the one that gets closest to it. When the
    // obstacle is already inside the bubble every candidate has the same worst case (now), so
    // break the tie on which one opens the gap fastest -- stopping next to it is not an escape.
    result.conflict_free = false;
    constexpr double kSame = 0.01;
    for (const auto & c : candidates) {
      if (best == nullptr || c.margin > best->margin + kSame ||
        (std::abs(c.margin - best->margin) <= kSame &&
        (c.escape > best->escape + kSame ||
        (std::abs(c.escape - best->escape) <= kSame && c.cost < best->cost))))
      {
        best = &c;
      }
    }
  }
  result.velocity = best->v;
  result.min_separation = best->margin;
  return result;
}

}  // namespace autolivery_core
