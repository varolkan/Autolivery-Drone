#include "autolivery_core/path_follower.hpp"

#include <utility>

namespace autolivery_core
{

void PathFollower::reset(Path path)
{
  path_ = std::move(path);
  total_length_ = pathLength(path_);
  segment_ = 0;
}

PathFollower::Progress PathFollower::update(
  const Vec3 & position, double lookahead, double acceptance)
{
  Progress p;
  if (path_.empty()) {
    p.finished = true;
    return p;
  }
  if (path_.size() == 1) {
    p.projection = p.carrot = p.next_waypoint = path_.front();
    p.remaining = distance(position, path_.front());
    p.cross_track = p.remaining;
    p.final_segment = true;
    p.finished = p.remaining <= acceptance;
    return p;
  }

  const std::size_t last_segment = path_.size() - 2;
  while (segment_ < last_segment) {
    const Vec3 & a = path_[segment_];
    const Vec3 & b = path_[segment_ + 1];
    const Vec3 & c = path_[segment_ + 2];
    const double t = projectOntoSegment(position, a, b);
    if (t >= 1.0 || distance(position, b) < acceptance) {
      ++segment_;
      continue;
    }
    // Cut the corner: already past the start of the next leg and closer to it than to this
    // one. Without this a vehicle that turns wide (a plane, or anything pushed off the path by
    // avoidance) can join the next leg without ever passing the end of this one or coming
    // within `acceptance` of the corner, and then the carrot keeps dragging it back.
    // "Closer" has to be by a real margin: where a route doubles back on itself the two legs
    // lie on top of each other, and a tie decided by rounding would skip the whole way out.
    constexpr double kCloserBy = 1.0;
    const double t_next = projectOntoSegment(position, b, c);
    if (t_next > 0.0 &&
      distance(position, lerp(b, c, t_next)) + kCloserBy < distance(position, lerp(a, b, t)))
    {
      ++segment_;
      continue;
    }
    break;
  }

  const Vec3 & a = path_[segment_];
  const Vec3 & b = path_[segment_ + 1];
  p.segment = segment_;
  p.final_segment = segment_ == last_segment;
  p.next_waypoint = b;
  p.projection = lerp(a, b, projectOntoSegment(position, a, b));
  p.cross_track = distance(position, p.projection);

  p.remaining = distance(p.projection, b);
  for (std::size_t i = segment_ + 1; i + 1 < path_.size(); ++i) {
    p.remaining += distance(path_[i], path_[i + 1]);
  }

  // Walk `lookahead` metres along the path from the projection.
  Vec3 carrot = p.projection;
  double left = lookahead;
  for (std::size_t i = segment_; i + 1 < path_.size() && left > 0.0; ++i) {
    const Vec3 & to = path_[i + 1];
    const double d = distance(carrot, to);
    if (d >= left) {
      carrot = carrot + (to - carrot) * (left / d);
      left = 0.0;
    } else {
      left -= d;
      carrot = to;
    }
  }
  p.carrot = carrot;
  p.finished = p.final_segment && distance(position, path_.back()) <= acceptance;
  return p;
}

}  // namespace autolivery_core
