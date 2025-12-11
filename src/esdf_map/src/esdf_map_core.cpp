#include "esdf_map/esdf_map_core.hpp"
#include "esdf_map/utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace esdf_map {

    struct EsdfMapCore::Impl {
        std::vector<float> distance_grid;
        std::vector<float> weight_grid;
        std::vector<uint8_t> observed_grid;

        UpdateRegion pending_region;
        int num_voxels = 0;
    };

    EsdfMapCore::EsdfMapCore(const Config &config) : config_(config) {
        if (config_.resolution <= 0.0) {
            throw std::runtime_error("EsdfMapCore: resolution must be > 0.");
        }
        if (config_.dims.x() <= 0 || config_.dims.y() <= 0 || config_.dims.z() <= 0) {
            throw std::runtime_error("EsdfMapCore: dims must be > 0 in all axes.");
        }

        impl_ = std::make_unique<Impl>();
        impl_->num_voxels = config_.dims.x() * config_.dims.y() * config_.dims.z();
        impl_->distance_grid.resize(impl_->num_voxels);
        impl_->weight_grid.resize(impl_->num_voxels);
        impl_->observed_grid.resize(impl_->num_voxels);

        clear();
    }

    EsdfMapCore::~EsdfMapCore() = default;

    void EsdfMapCore::clear() {
        const float max_dist = static_cast<float>(config_.esdf_max_distance);
        std::fill(impl_->distance_grid.begin(), impl_->distance_grid.end(), max_dist);
        std::fill(impl_->weight_grid.begin(), impl_->weight_grid.end(), 0.0f);
        std::fill(impl_->observed_grid.begin(), impl_->observed_grid.end(), 0u);

        impl_->pending_region.valid = false;
    }

    // tmp
    const EsdfMapCore::Config &EsdfMapCore::config() const {
        return config_;
    }

    // tmp
    MapInfo EsdfMapCore::getMapInfo() const {
        MapInfo info;
        info.resolution = config_.resolution;
        info.origin = config_.origin;
        info.dims = config_.dims;
        return info;
    }

    // Simple helper: world position inside map bounds?
    bool EsdfMapCore::isInside(const Vec3d &p_M) const {
        Vec3i idx = worldToIndex(p_M, config_.origin, config_.resolution);
        return isInsideIndex(idx, config_.dims);
    }

    bool EsdfMapCore::isObserved(const Vec3d &p_M) const {
        Vec3i idx = worldToIndex(p_M, config_.origin, config_.resolution);
        if (!isInsideIndex(idx, config_.dims)) return false;
        int linear = flattenIndex(idx, config_.dims);
        return impl_->observed_grid[linear] != 0u;
    }

    // Internal: sample ESDF at world position using trilinear interpolation.
    static bool sampleTrilinear(const EsdfMapCore::Config &cfg,
                                const EsdfMapCore::Impl &impl,
                                const Vec3d &p_M,
                                double &distance_out) {
        Vec3d g = worldToGrid(p_M, cfg.origin, cfg.resolution);

        const double gx = g.x();
        const double gy = g.y();
        const double gz = g.z();

        int ix = static_cast<int>(std::floor(gx));
        int iy = static_cast<int>(std::floor(gy));
        int iz = static_cast<int>(std::floor(gz));

        const int nx = cfg.dims.x();
        const int ny = cfg.dims.y();
        const int nz = cfg.dims.z();

        if (nx < 2 || ny < 2 || nz < 2) {
            return false;
        }

        // Clamp indices so that ix+1, etc. stay inside.
        ix = clamp(ix, 0, nx - 2);
        iy = clamp(iy, 0, ny - 2);
        iz = clamp(iz, 0, nz - 2);

        const double fx = clamp(gx - ix, 0.0, 1.0);
        const double fy = clamp(gy - iy, 0.0, 1.0);
        const double fz = clamp(gz - iz, 0.0, 1.0);

        const double fx0 = 1.0 - fx;
        const double fy0 = 1.0 - fy;
        const double fz0 = 1.0 - fz;

        const Vec3i base(ix, iy, iz);

        auto d_at = [&](int dx, int dy, int dz) -> float {
            Vec3i idx = base + Vec3i(dx, dy, dz);
            if (!isInsideIndex(idx, cfg.dims)) {
                return static_cast<float>(cfg.esdf_max_distance);
            }
            int linear = flattenIndex(idx, cfg.dims);
            return impl.distance_grid[linear];
        };

        const double d000 = d_at(0, 0, 0);
        const double d100 = d_at(1, 0, 0);
        const double d010 = d_at(0, 1, 0);
        const double d110 = d_at(1, 1, 0);
        const double d001 = d_at(0, 0, 1);
        const double d101 = d_at(1, 0, 1);
        const double d011 = d_at(0, 1, 1);
        const double d111 = d_at(1, 1, 1);

        const double w000 = fx0 * fy0 * fz0;
        const double w100 = fx * fy0 * fz0;
        const double w010 = fx0 * fy * fz0;
        const double w110 = fx * fy * fz0;
        const double w001 = fx0 * fy0 * fz;
        const double w101 = fx * fy0 * fz;
        const double w011 = fx0 * fy * fz;
        const double w111 = fx * fy * fz;

        distance_out =
            w000 * d000 + w100 * d100 + w010 * d010 + w110 * d110 +
            w001 * d001 + w101 * d101 + w011 * d011 + w111 * d111;

        return true;
    }

    bool EsdfMapCore::queryDistance(const Vec3d &p_M, double &distance) const {
        if (!isInside(p_M)) return false;
        return sampleTrilinear(config_, *impl_, p_M, distance);
    }

    // tmp
    bool EsdfMapCore::queryDistanceAndGradient(const Vec3d &p_M,
                                               double &distance,
                                               Vec3d &gradient) const {
        if (!isInside(p_M)) return false;
        bool ok = sampleTrilinear(config_, *impl_, p_M, distance);
        if (!ok) return false;

        const double eps = config_.resolution;
        const Vec3d ex(eps, 0.0, 0.0);
        const Vec3d ey(0.0, eps, 0.0);
        const Vec3d ez(0.0, 0.0, eps);

        auto safe_sample = [&](const Vec3d &p, double &d_out) -> bool {
            if (!isInside(p)) return false;
            return sampleTrilinear(config_, *impl_, p, d_out);
        };

        double dxp, dxm, dyp, dym, dzp, dzm;
        bool ok_xp = safe_sample(p_M + ex, dxp);
        bool ok_xm = safe_sample(p_M - ex, dxm);
        bool ok_yp = safe_sample(p_M + ey, dyp);
        bool ok_ym = safe_sample(p_M - ey, dym);
        bool ok_zp = safe_sample(p_M + ez, dzp);
        bool ok_zm = safe_sample(p_M - ez, dzm);

        gradient.setZero();

        if (ok_xp && ok_xm) gradient.x() = (dxp - dxm) / (2.0 * eps);
        else if (ok_xp)     gradient.x() = (dxp - distance) / eps;
        else if (ok_xm)     gradient.x() = (distance - dxm) / eps;

        if (ok_yp && ok_ym) gradient.y() = (dyp - dym) / (2.0 * eps);
        else if (ok_yp)     gradient.y() = (dyp - distance) / eps;
        else if (ok_ym)     gradient.y() = (distance - dym) / eps;

        if (ok_zp && ok_zm) gradient.z() = (dzp - dzm) / (2.0 * eps);
        else if (ok_zp)     gradient.z() = (dzp - distance) / eps;
        else if (ok_zm)     gradient.z() = (distance - dzm) / eps;

        return true;
    }

    void EsdfMapCore::batchQueryDistance(const Mat3xN &positions_M,
                                         Eigen::VectorXd &distances,
                                         Eigen::VectorXi &observed) const {
        const int n = static_cast<int>(positions_M.cols());
        distances.resize(n);
        observed.resize(n);

        for (int i = 0; i < n; ++i) {
            double d;
            Vec3d p = positions_M.col(i);
            bool ok = queryDistance(p, d);
            if (ok) {
                distances[i] = d;
                observed[i] = isObserved(p) ? 1 : 0;
            } else {
                distances[i] = config_.esdf_max_distance;
                observed[i] = 0;
            }
        }
    }

    void EsdfMapCore::batchQueryDistanceAndGradient(const Mat3xN &positions_M,
                                                    Eigen::VectorXd &distances,
                                                    Mat3xN &gradients,
                                                    Eigen::VectorXi &observed) const {
        const int n = static_cast<int>(positions_M.cols());
        distances.resize(n);
        gradients.resize(Eigen::NoChange, n);
        observed.resize(n);

        for (int i = 0; i < n; ++i) {
            Vec3d g;
            double d;
            Vec3d p = positions_M.col(i);
            bool ok = queryDistanceAndGradient(p, d, g);
            if (ok) {
                distances[i] = d;
                gradients.col(i) = g;
                observed[i] = isObserved(p) ? 1 : 0;
            } else {
                distances[i] = config_.esdf_max_distance;
                gradients.col(i).setZero();
                observed[i] = 0;
            }
        }
    }

    void EsdfMapCore::getAllVoxels(std::vector<Voxel> &voxels) const {
        voxels.clear();
        voxels.resize(static_cast<size_t>(impl_->num_voxels));

        int linear = 0;
        for (int z = 0; z < config_.dims.z(); ++z) {
            for (int y = 0; y < config_.dims.y(); ++y) {
                for (int x = 0; x < config_.dims.x(); ++x, ++linear) {
                    Eigen::Vector3i idx(x, y, z);
                    Voxel &v = voxels[linear];
                    v.center   = indexToCenter(idx, config_.origin, config_.resolution);
                    v.distance = impl_->distance_grid[linear];
                    v.weight   = impl_->weight_grid[linear];
                    v.observed = impl_->observed_grid[linear] != 0u;
                }
            }
        }
    }

    void EsdfMapCore::extractSlice(double z_M, Slice2D &slice) const {
        const double gz = (z_M - config_.origin.z()) / config_.resolution;
        int iz = static_cast<int>(std::round(gz));

        const int nx = config_.dims.x();
        const int ny = config_.dims.y();
        const int nz = config_.dims.z();

        if (iz < 0 || iz >= nz) {
            slice.resolution = config_.resolution;
            slice.origin.setZero();
            slice.dims.setZero();
            slice.distances.clear();
            return;
        }

        slice.resolution = config_.resolution;
        slice.origin = Eigen::Vector2d(config_.origin.x(), config_.origin.y());
        slice.dims = Eigen::Vector2i(nx, ny);
        slice.distances.resize(static_cast<size_t>(nx * ny));

        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                Eigen::Vector3i idx(x, y, iz);
                int index_3d = flattenIndex(idx, config_.dims);
                int index_2d = y * nx + x;
                slice.distances[index_2d] = impl_->distance_grid[index_3d];
            }
        }
    }

    UpdateRegion EsdfMapCore::consumeUpdateRegion() {
        UpdateRegion out = impl_->pending_region;
        impl_->pending_region.valid = false;
        return out;
    }

    void EsdfMapCore::integrateCloud(const PointCloud &cloud_M, const Eigen::Isometry3d &T_M_L) {
        (void)T_M_L; // currently unused; kept for future raycasting.

        Vec3i min_idx(std::numeric_limits<int>::max(),
                      std::numeric_limits<int>::max(),
                      std::numeric_limits<int>::max());
        Vec3i max_idx(-1, -1, -1);
        bool any = false;

        for (const auto &p : cloud_M.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }

            Vec3d p_M(p.x, p.y, p.z);
            Vec3i idx = worldToIndex(p_M, config_.origin, config_.resolution);
            if (!isInsideIndex(idx, config_.dims)) {
                continue;
            }

            int linear = flattenIndex(idx, config_.dims);

            impl_->distance_grid[linear] = 0.0f; // mark as obstacle
            impl_->weight_grid[linear] += 1.0f;
            impl_->observed_grid[linear] = 1u;

            if (!any) {
                min_idx = max_idx = idx;
                any = true;
            } else {
                min_idx = min_idx.cwiseMin(idx);
                max_idx = max_idx.cwiseMax(idx);
            }
        }

        if (any) {
            if (!impl_->pending_region.valid) {
                impl_->pending_region.min_index = min_idx;
                impl_->pending_region.max_index = max_idx;
                impl_->pending_region.valid = true;
            }
            else {
                impl_->pending_region.min_index =
                    impl_->pending_region.min_index.cwiseMin(min_idx);
                impl_->pending_region.max_index =
                    impl_->pending_region.max_index.cwiseMax(max_idx);
            }
        }
    }

    // Multi-source BFS distance transform from obstacle voxels.
    void EsdfMapCore::updateEsdf() {
        const int nx = config_.dims.x();
        const int ny = config_.dims.y();
        const int nz = config_.dims.z();

        const float max_dist = static_cast<float>(config_.esdf_max_distance);
        const float inf = max_dist + 2.0f * static_cast<float>(config_.resolution);

        std::queue<int> q;

        for (int i = 0; i < impl_->num_voxels; ++i) {
            if (impl_->distance_grid[i] == 0.0f) {
                q.push(i);
            } else {
                impl_->distance_grid[i] = inf;
            }
        }

        auto push_neighbor = [&](int x, int y, int z, int cur_idx) {
            if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) {
                return;
            }
            Eigen::Vector3i idx_3d(x, y, z);
            int idx = flattenIndex(idx_3d, config_.dims);
            float tentative = impl_->distance_grid[cur_idx] + static_cast<float>(config_.resolution);
            if (tentative < impl_->distance_grid[idx] && tentative <= max_dist) {
                impl_->distance_grid[idx] = tentative;
                q.push(idx);
            }
        };

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            int z = cur / (nx * ny);
            int rem = cur - z * nx * ny;
            int y = rem / nx;
            int x = rem - y * nx;

            push_neighbor(x + 1, y, z, cur);
            push_neighbor(x - 1, y, z, cur);
            push_neighbor(x, y + 1, z, cur);
            push_neighbor(x, y - 1, z, cur);
            push_neighbor(x, y, z + 1, cur);
            push_neighbor(x, y, z - 1, cur);
        }

        for (int i = 0; i < impl_->num_voxels; ++i) {
            if (impl_->distance_grid[i] > max_dist) {
                impl_->distance_grid[i] = max_dist;
            }
        }

        impl_->pending_region.min_index = Vec3i(0, 0, 0);
        impl_->pending_region.max_index = Vec3i(nx - 1, ny - 1, nz - 1);
        impl_->pending_region.valid = true;
    }

} // namespace esdf_map
