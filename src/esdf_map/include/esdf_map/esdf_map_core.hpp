#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "esdf_map/esdf_map_primitives.hpp"
#include "esdf_map/voxel_view.hpp"

namespace esdf_map
{
    class EsdfMapCore {
    public:
        struct Config {
            // Map geometry (in some global frame, e.g. map_origin).
            double resolution = 0.1;      // voxel size [m]
            Vec3d origin = Vec3d::Zero(); // min corner of voxel (0,0,0) in map frame
            Vec3i dims = Vec3i::Zero();   // number of voxels in x,y,z

            // TSDF / occupancy integration.
            double max_ray_length = 30.0;     // [m] //TODO: Raycast not implemented
            double truncation_distance = 0.3; // [m] //TODO: Raycast not implemented
            double esdf_max_distance = 5.0;   // [m]
            bool enable_clearing_rays = true;        //TODO: Raycast not implemented
            bool enable_chamfer_relax = true; // for accurate result

            // Probabilistic fusion (if you choose log-odds style).
            double p_hit = 0.70;                     //TODO: Raycast not implemented
            double p_miss = 0.35;                    //TODO: Raycast not implemented
            double p_min = 0.12;                     //TODO: Raycast not implemented
            double p_max = 0.97;                     //TODO: Raycast not implemented
            double p_occ = 0.80;                     //TODO: Raycast not implemented
        };
        
        explicit EsdfMapCore(const Config &config);
        ~EsdfMapCore();

        // cloud_M: points in map frame (same frame as Config::origin).
        // T_M_L: lidar pose in map frame.
        void integrateCloud(const PointCloud &cloud_M, const Eigen::Isometry3d &T_M_L);

        // Rebuild / incrementally update ESDF from TSDF / occupancy.
        void updateEsdf();
        void clear();

        const Config &config() const;
        MapInfo getMapInfo() const;

        bool isInside(const Vec3d &p_M) const;
        bool isObserved(const Vec3d &p_M) const;

        bool queryDistance(const Vec3d &p_M, double &distance) const;
        bool queryDistanceAndGradient(const Vec3d &p_M, double &distance, Vec3d &gradient) const;
        void batchQueryDistance(const Mat3xN &positions_M, Eigen::VectorXd &distances, Eigen::VectorXi &observed) const;
        void batchQueryDistanceAndGradient(const Mat3xN &positions_M, Eigen::VectorXd &distances, Mat3xN &gradients, Eigen::VectorXi &observed) const;

        // Export all ESDF voxels (e.g. for /esdf/grid).
        VoxelRegionView getVoxelView(const UpdateRegion& region) const;
        VoxelRegionView getAllVoxels() const;

        // Extract a z-slice of ESDF for 2D costmap building.
        void extractSlice(double z_M, Slice2D &slice) const;

        // Region that changed since last call (for incremental publishing).
        UpdateRegion consumeUpdateRegion();

        struct Impl;

    private:
        Config config_;
        std::unique_ptr<Impl> impl_;
    };

} // namespace esdf_map
