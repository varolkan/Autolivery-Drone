// Post-processing for planner output.
//
// Grid and sampling planners both produce jagged paths: A* staircases along voxel diagonals,
// RRT* zig-zags between random samples. Shortcutting removes every waypoint that has a clear
// line of sight past it, which for a city route typically collapses a few hundred points into
// a handful of straight legs around building corners. Densifying then splits long legs so a
// waypoint-following autopilot always has a nearby target.

#ifndef AUTOLIVERY_CORE__PATH_SMOOTHER_HPP_
#define AUTOLIVERY_CORE__PATH_SMOOTHER_HPP_

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

// Greedy line-of-sight pruning. Keeps the first and last points; every output segment is
// clear of `map` with `clearance`. Segments of the input that were not clear to begin with
// are kept as-is rather than silently "fixed".
Path shortcutPath(const Path & path, const WorldMap & map, double clearance);

// Splits segments longer than `max_segment` into equal pieces.
Path densifyPath(const Path & path, double max_segment);

// Drops consecutive points closer than `min_spacing` (keeps the endpoints).
Path removeDuplicates(const Path & path, double min_spacing);

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__PATH_SMOOTHER_HPP_
