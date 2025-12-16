#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace esdf_map
{
    using Vec3d = Eigen::Vector3d;
    using Vec3i = Eigen::Vector3i;
    using Mat3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    struct MapInfo {
        double resolution;
        Vec3d origin;
        Vec3i dims;
    };

    struct Slice2D {
        double resolution = 0.0;
        Eigen::Vector2d origin = Eigen::Vector2d::Zero(); // min corner
        Eigen::Vector2i dims = Eigen::Vector2i::Zero();   // nx, ny
        std::vector<float> distances;                     // row-major [y * nx + x]
    };

    struct UpdateRegion {
        Vec3i min_index = Vec3i::Zero(); // inclusive
        Vec3i max_index = Vec3i::Zero(); // inclusive
        bool valid = false;

        inline void clamp(const Vec3i& min_c, const Vec3i& max_c) {
            min_index = min_index.cwiseMax(min_c);
            max_index = max_index.cwiseMin(max_c);
        }

        inline void expand(int margin_vox) {
            if (!valid) return;
            min_index.array() -= margin_vox;
            max_index.array() += margin_vox;
        }
    };

} // namespace esdf_map

