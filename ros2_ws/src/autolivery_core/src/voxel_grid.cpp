#include "autolivery_core/voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autolivery_core
{

namespace
{

// Upper bound on grid size. A 400 m x 400 m x 120 m city at 1 m is 19.2 M cells; anything
// past this is almost certainly a units mistake in the map (millimetres, or a missing
// decimal point) and would otherwise take the machine down allocating planner arrays.
constexpr std::size_t kMaxCells = 60'000'000;

}  // namespace

VoxelGrid::VoxelGrid(const WorldMap & map, double resolution, double clearance)
: origin_(map.boundsMin()), resolution_(resolution), clearance_(clearance)
{
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("voxel resolution must be positive");
  }
  if (clearance < 0.0) {
    throw std::invalid_argument("clearance must not be negative");
  }
  const Vec3 extent = map.boundsMax() - map.boundsMin();
  nx_ = std::max(1, static_cast<int>(std::ceil(extent.x / resolution)));
  ny_ = std::max(1, static_cast<int>(std::ceil(extent.y / resolution)));
  nz_ = std::max(1, static_cast<int>(std::ceil(extent.z / resolution)));
  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_ * nz_;
  if (cells > kMaxCells) {
    throw std::invalid_argument(
            "voxel grid would have " + std::to_string(cells) +
            " cells; increase the resolution or shrink the geofence");
  }
  blocked_.assign(cells, 0);

  const double grow = clearance + 0.5 * std::sqrt(3.0) * resolution;
  for (const auto & b : map.buildings()) {
    rasterise(b.min, b.max, nullptr, grow);
  }
  for (const auto & z : map.noFlyZones()) {
    rasterise(z.min, z.max, nullptr, grow);
  }
  for (const auto & c : map.cylinders()) {
    const Vec3 lo{c.base.x - c.radius, c.base.y - c.radius, c.base.z};
    const Vec3 hi{c.base.x + c.radius, c.base.y + c.radius, c.base.z + c.height};
    rasterise(lo, hi, &c, grow);
  }

  // The geofence is a hard wall: voxels whose centre sits outside it (the last partial layer
  // when the extent is not a multiple of the resolution) are unusable.
  for (std::size_t f = 0; f < blocked_.size(); ++f) {
    if (!map.inBounds(toWorld(unflatten(f)))) {
      blocked_[f] = 1;
    }
  }
}

void VoxelGrid::rasterise(const Vec3 & lo, const Vec3 & hi, const Cylinder * cylinder, double grow)
{
  const GridIndex a = toIndex(lo - Vec3{grow, grow, grow});
  const GridIndex b = toIndex(hi + Vec3{grow, grow, grow});
  const int x0 = std::max(0, a.x);
  const int y0 = std::max(0, a.y);
  const int z0 = std::max(0, a.z);
  const int x1 = std::min(nx_ - 1, b.x);
  const int y1 = std::min(ny_ - 1, b.y);
  const int z1 = std::min(nz_ - 1, b.z);

  for (int z = z0; z <= z1; ++z) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const GridIndex idx{x, y, z};
        const Vec3 c = toWorld(idx);
        bool hit;
        if (cylinder != nullptr) {
          const double r = cylinder->radius + grow;
          hit = c.z >= cylinder->base.z - grow &&
            c.z <= cylinder->base.z + cylinder->height + grow &&
            std::hypot(c.x - cylinder->base.x, c.y - cylinder->base.y) <= r;
        } else {
          hit = c.x >= lo.x - grow && c.x <= hi.x + grow &&
            c.y >= lo.y - grow && c.y <= hi.y + grow &&
            c.z >= lo.z - grow && c.z <= hi.z + grow;
        }
        if (hit) {
          blocked_[flatten(idx)] = 1;
        }
      }
    }
  }
}

GridIndex VoxelGrid::toIndex(const Vec3 & p) const
{
  const Vec3 rel = (p - origin_) / resolution_;
  return {
    static_cast<int>(std::floor(rel.x)),
    static_cast<int>(std::floor(rel.y)),
    static_cast<int>(std::floor(rel.z))};
}

Vec3 VoxelGrid::toWorld(const GridIndex & i) const
{
  return origin_ + Vec3{
    (i.x + 0.5) * resolution_,
    (i.y + 0.5) * resolution_,
    (i.z + 0.5) * resolution_};
}

std::optional<GridIndex> VoxelGrid::nearestFree(const GridIndex & from, int max_radius) const
{
  if (isFree(from)) {
    return from;
  }
  const Vec3 target = toWorld(from);
  for (int r = 1; r <= max_radius; ++r) {
    std::optional<GridIndex> best;
    double best_d = std::numeric_limits<double>::infinity();
    // Walk only the shell at Chebyshev distance r.
    for (int dz = -r; dz <= r; ++dz) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
            continue;
          }
          const GridIndex cand{from.x + dx, from.y + dy, from.z + dz};
          if (!isFree(cand)) {
            continue;
          }
          const double d = distance(toWorld(cand), target);
          if (d < best_d) {
            best_d = d;
            best = cand;
          }
        }
      }
    }
    if (best) {
      return best;
    }
  }
  return std::nullopt;
}

std::size_t VoxelGrid::blockedCount() const
{
  return static_cast<std::size_t>(std::count(blocked_.begin(), blocked_.end(), 1));
}

}  // namespace autolivery_core
