// A 3D occupancy grid rasterised from a WorldMap, for grid planners.
//
// A voxel is marked blocked if any obstacle, grown by `clearance + half the voxel diagonal`,
// touches it. The extra half-diagonal is what makes grid paths safe without re-checking them:
// every point on the segment between two adjacent free voxel centres is within half a
// diagonal of one of them, so the whole segment keeps at least `clearance` from obstacles.

#ifndef AUTOLIVERY_CORE__VOXEL_GRID_HPP_
#define AUTOLIVERY_CORE__VOXEL_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "autolivery_core/geometry.hpp"
#include "autolivery_core/world_map.hpp"

namespace autolivery_core
{

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & o) const {return x == o.x && y == o.y && z == o.z;}
};

class VoxelGrid
{
public:
  VoxelGrid(const WorldMap & map, double resolution, double clearance);

  double resolution() const {return resolution_;}
  double clearance() const {return clearance_;}
  int sizeX() const {return nx_;}
  int sizeY() const {return ny_;}
  int sizeZ() const {return nz_;}
  std::size_t cellCount() const {return blocked_.size();}

  bool contains(const GridIndex & i) const
  {
    return i.x >= 0 && i.y >= 0 && i.z >= 0 && i.x < nx_ && i.y < ny_ && i.z < nz_;
  }

  bool isFree(const GridIndex & i) const {return contains(i) && !blocked_[flatten(i)];}

  std::size_t flatten(const GridIndex & i) const
  {
    return (static_cast<std::size_t>(i.z) * ny_ + i.y) * nx_ + i.x;
  }

  GridIndex unflatten(std::size_t f) const
  {
    const int x = static_cast<int>(f % nx_);
    const int y = static_cast<int>((f / nx_) % ny_);
    const int z = static_cast<int>(f / (static_cast<std::size_t>(nx_) * ny_));
    return {x, y, z};
  }

  GridIndex toIndex(const Vec3 & p) const;
  Vec3 toWorld(const GridIndex & i) const;  // voxel centre

  // Closest free voxel to `from` within `max_radius` voxels (Chebyshev shells), if any.
  std::optional<GridIndex> nearestFree(const GridIndex & from, int max_radius) const;

  std::size_t blockedCount() const;

private:
  void rasterise(const Vec3 & lo, const Vec3 & hi, const Cylinder * cylinder, double grow);

  Vec3 origin_;
  double resolution_;
  double clearance_;
  int nx_{0};
  int ny_{0};
  int nz_{0};
  std::vector<std::uint8_t> blocked_;
};

}  // namespace autolivery_core

#endif  // AUTOLIVERY_CORE__VOXEL_GRID_HPP_
