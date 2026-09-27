#include "autolivery_core/path_smoother.hpp"

#include <cmath>
#include <cstddef>

namespace autolivery_core
{

Path shortcutPath(const Path & path, const WorldMap & map, double clearance)
{
  if (path.size() <= 2) {
    return path;
  }
  Path out{path.front()};
  std::size_t i = 0;
  while (i < path.size() - 1) {
    // Farthest point visible from i. Scanning from the end finds the longest shortcut first,
    // which is what we want; the path lengths involved (hundreds of points) keep the
    // quadratic worst case in the low milliseconds.
    std::size_t next = i + 1;
    for (std::size_t j = path.size() - 1; j > i + 1; --j) {
      if (map.segmentClear(path[i], path[j], clearance)) {
        next = j;
        break;
      }
    }
    out.push_back(path[next]);
    i = next;
  }
  return out;
}

Path densifyPath(const Path & path, double max_segment)
{
  if (path.size() < 2 || !(max_segment > 0.0)) {
    return path;
  }
  Path out{path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double len = distance(path[i - 1], path[i]);
    const int pieces = std::max(1, static_cast<int>(std::ceil(len / max_segment)));
    for (int k = 1; k <= pieces; ++k) {
      out.push_back(lerp(path[i - 1], path[i], static_cast<double>(k) / pieces));
    }
  }
  return out;
}

Path removeDuplicates(const Path & path, double min_spacing)
{
  if (path.size() <= 2) {
    return path;
  }
  Path out{path.front()};
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    if (distance(out.back(), path[i]) >= min_spacing) {
      out.push_back(path[i]);
    }
  }
  if (distance(out.back(), path.back()) < min_spacing && out.size() > 1) {
    out.back() = path.back();
  } else {
    out.push_back(path.back());
  }
  return out;
}

}  // namespace autolivery_core
