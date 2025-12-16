#pragma once
#include <cstdint>
#include <cstddef>
#include <Eigen/Core>
#include "esdf_map/esdf_map_primitives.hpp"

namespace esdf_map {

    struct VoxelSample {
        float x, y, z; // center
        float distance = 0.f;
        float weight = 0.f;
        bool observed = false;
    };


    struct VoxelRegionView {
        double resolution = 0.0;
        Vec3d origin = Vec3d::Zero();
        Vec3i dims = Vec3i::Zero();
        UpdateRegion region;

        const float* distance = nullptr;
        const float* weight = nullptr;
        const uint8_t* observed = nullptr;

        std::size_t count() const {
            if (!region.valid) return 0;
            const auto d = region.max_index - region.min_index + Vec3i(1,1,1);
            if ((d.array() <= 0).any()) return 0;
            return (std::size_t)d.x() * (std::size_t)d.y() * (std::size_t)d.z();
        }

        template <class F>
        void forEach(F&& f) const {
            if (!region.valid || !distance || !weight || !observed) return;

            const int nx = dims.x();
            const int ny = dims.y();
            const int stride_x = 1;
            const int stride_y = nx;
            const int stride_z = nx * ny;

            const float res = (float)resolution;
            const float ox = (float)origin.x();
            const float oy = (float)origin.y();
            const float oz = (float)origin.z();

            const int x0 = region.min_index.x(), x1 = region.max_index.x();
            const int y0 = region.min_index.y(), y1 = region.max_index.y();
            const int z0 = region.min_index.z(), z1 = region.max_index.z();

            for (int z = z0; z <= z1; ++z) {
                const float cz = oz + (z + 0.5f) * res;
                for (int y = y0; y <= y1; ++y) {
                    const float cy = oy + (y + 0.5f) * res;

                    int linear = z * stride_z + y * stride_y + x0 * stride_x;
                    float cx = ox + (x0 + 0.5f) * res;

                    for (int x = x0; x <= x1; ++x, ++linear, cx += res) {
                        VoxelSample v;
                        v.x = cx; v.y = cy; v.z = cz;
                        v.distance = distance[linear];
                        v.weight = weight[linear];
                        v.observed = observed[linear] != 0u;
                        f(v);
                    }
                }
            }
        }
    };

} // namespace esdf_map
