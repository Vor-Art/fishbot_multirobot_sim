#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace esdf_map
{
    // Clamp helper.
    template <typename T>
    inline T clamp(const T &v, const T &lo, const T &hi) {
        return std::max(lo, std::min(v, hi));
    }

    // Check if integer index is inside [0, dims-1].
    inline bool isInsideIndex(const Vec3i &idx, const Vec3i &dims) {
        return (idx.x() >= 0 && idx.x() < dims.x() &&
                idx.y() >= 0 && idx.y() < dims.y() &&
                idx.z() >= 0 && idx.z() < dims.z());
    }

    // Flatten 3D index to 1D row-major index.
    inline int flattenIndex(const Vec3i &idx, const Vec3i &dims) {
        return (idx.z() * dims.y() + idx.y()) * dims.x() + idx.x();
    }

    // Convert world position to continuous voxel coordinates.
    inline Vec3d worldToGrid(const Vec3d &p_M,
                             const Vec3d &origin,
                             float resolution)
    {
        return (p_M - origin) / resolution;
    }

    // Integer voxel index from world position (no bounds check).
    inline Vec3i worldToIndex(const Vec3d &p_M,
                              const Vec3d &origin,
                              float resolution)
    {
        Vec3d g = worldToGrid(p_M, origin, resolution);
        return Vec3i(
            static_cast<int>(std::floor(g.x())),
            static_cast<int>(std::floor(g.y())),
            static_cast<int>(std::floor(g.z())));
    }

    // Center of voxel in world coordinates.
    inline Vec3d indexToCenter(const Vec3i &idx,
                               const Vec3d &origin,
                               float resolution)
    {
        return origin + (idx.cast<double>() + Vec3d::Constant(0.5)) * resolution;
    }

} // namespace esdf_map
