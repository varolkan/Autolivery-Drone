// Tracks progress along a planned path and produces a lookahead ("carrot") target.
//
// Chasing a point a fixed distance ahead of the vehicle's projection onto the path, rather
// than the next waypoint, gives smooth corners and pulls the vehicle back onto the path after
// an avoidance manoeuvre pushed it off: the carrot stays on the path, so the vehicle heads
// for the path, not for a waypoint it may already have passed.

#ifndef AUTOLIVERY_CORE__PATH_FOLLOWER_HPP_
#define AUTOLIVERY_CORE__PATH_FOLLOWER_HPP_

#include <cstddef>

#include "autolivery_core/geometry.hpp"

namespace autolivery_core
{

class PathFollower
{
public:
  struct Progress
  {
    Vec3 projection;      // closest point on the current segment
    Vec3 carrot;          // lookahead target on the path
    Vec3 next_waypoint;   // end of the current segment
    double remaining{0.0};     // path length from the projection to the end
    double cross_track{0.0};   // distance from the vehicle to the projection
    std::size_t segment{0};
    bool final_segment{false};
    bool finished{false};      // on the final segment and within acceptance of the end
  };

  void reset(Path path);
  const Path & path() const {return path_;}
  bool empty() const {return path_.empty();}
  double totalLength() const {return total_length_;}

  // Advances along the path given the vehicle position. `acceptance` both switches segments
  // early (so the vehicle does not have to touch every corner) and decides when the path is done.
  Progress update(const Vec3 & position, double lookahead, double acceptance);

private:
  Path path_;
  double total_length_{0.0};
  std::size_t segment_{0};
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__PATH_FOLLOWER_HPP_
