// Small value types shared by every planner and controller in autolivery_core.
//
// Everything in this library is expressed in the *world* frame: ENU, metres, origin at the
// city map origin (which is also the Gazebo world origin). Converting to and from the
// autopilot's local frame is the ROS layer's job, not ours -- keeping one frame here is what
// lets the mission logic be tested without a vehicle.

#ifndef AUTOLIVERY_CORE__GEOMETRY_HPP_
#define AUTOLIVERY_CORE__GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace autolivery_core
{

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3() = default;
  constexpr Vec3(double x_, double y_, double z_)
  : x(x_), y(y_), z(z_) {}

  constexpr Vec3 operator+(const Vec3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  constexpr Vec3 operator-(const Vec3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  constexpr Vec3 operator*(double s) const {return {x * s, y * s, z * s};}
  constexpr Vec3 operator/(double s) const {return {x / s, y / s, z / s};}
  constexpr Vec3 operator-() const {return {-x, -y, -z};}
  Vec3 & operator+=(const Vec3 & o) {x += o.x; y += o.y; z += o.z; return *this;}
  Vec3 & operator-=(const Vec3 & o) {x -= o.x; y -= o.y; z -= o.z; return *this;}
  Vec3 & operator*=(double s) {x *= s; y *= s; z *= s; return *this;}

  constexpr double dot(const Vec3 & o) const {return x * o.x + y * o.y + z * o.z;}
  double norm() const {return std::sqrt(dot(*this));}
  double horizontalNorm() const {return std::hypot(x, y);}
  constexpr Vec3 horizontal() const {return {x, y, 0.0};}

  // Unit vector, or zero for a (near) zero vector. Callers that care about the degenerate
  // case check norm() first; everyone else gets a harmless zero instead of NaN.
  Vec3 normalized() const
  {
    const double n = norm();
    return n > 1e-9 ? *this / n : Vec3{};
  }

  bool isFinite() const {return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);}
};

inline constexpr Vec3 operator*(double s, const Vec3 & v) {return v * s;}

inline double distance(const Vec3 & a, const Vec3 & b) {return (a - b).norm();}
inline double horizontalDistance(const Vec3 & a, const Vec3 & b) {return (a - b).horizontalNorm();}
inline Vec3 lerp(const Vec3 & a, const Vec3 & b, double t) {return a + (b - a) * t;}

// Limits the horizontal and vertical parts of a velocity independently, which is how
// multirotor autopilots express their envelope (MPC_XY_VEL_MAX vs MPC_Z_VEL_MAX_UP/DN).
inline Vec3 clampVelocity(const Vec3 & v, double max_horizontal, double max_up, double max_down)
{
  Vec3 out = v;
  const double h = v.horizontalNorm();
  if (h > max_horizontal && h > 1e-9) {
    out.x *= max_horizontal / h;
    out.y *= max_horizontal / h;
  }
  out.z = std::clamp(v.z, -max_down, max_up);
  return out;
}

// Wraps an angle to (-pi, pi].
inline double wrapAngle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

using Path = std::vector<Vec3>;

inline double pathLength(const Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += distance(path[i - 1], path[i]);
  }
  return length;
}

// Closest point to p on segment ab, returned as the interpolation parameter in [0, 1].
inline double projectOntoSegment(const Vec3 & p, const Vec3 & a, const Vec3 & b)
{
  const Vec3 ab = b - a;
  const double len_sq = ab.dot(ab);
  if (len_sq < 1e-12) {
    return 0.0;
  }
  return std::clamp((p - a).dot(ab) / len_sq, 0.0, 1.0);
}

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__GEOMETRY_HPP_
