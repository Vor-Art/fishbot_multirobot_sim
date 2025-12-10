#include "esdf_map/esdf_map_core.hpp"

#include <algorithm>
#include <cmath>

namespace esdf_map {

namespace {
constexpr double kMinResolution = 0.01;
constexpr double kMinSizeMeters = 0.01;
}

EsdfMapCore::EsdfMapCore() = default;

EsdfMapCore::EsdfMapCore(
    double default_distance, double costmap_resolution, double costmap_size_m) {
  setDefaultDistance(default_distance);
  setCostmap(costmap_resolution, costmap_size_m);
}

void EsdfMapCore::setDefaultDistance(double default_distance) {
  default_distance_ = std::max(0.0, default_distance);
}

void EsdfMapCore::setCostmap(double resolution, double size_m) {
  costmap_resolution_ = std::max(resolution, kMinResolution);
  costmap_size_m_ = std::max(size_m, kMinSizeMeters);
}

void EsdfMapCore::updateFromCloud(std::size_t point_count) {
  last_cloud_size_ = point_count;
}

Costmap2d EsdfMapCore::buildCostmap(std::size_t publish_count) const {
  Costmap2d costmap;
  costmap.resolution = costmap_resolution_;
  costmap.size_m = costmap_size_m_;

  const auto cells = static_cast<uint32_t>(
      std::max<double>(1.0, std::ceil(costmap_size_m_ / costmap_resolution_)));
  costmap.width = cells;
  costmap.height = cells;
  costmap.data.resize(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0);

  const double center = static_cast<double>(cells) / 2.0;
  for (std::size_t i = 0; i < costmap.data.size(); ++i) {
    const auto x = static_cast<double>(i % cells);
    const auto y = static_cast<double>(i / cells);
    const double dx = (x - center) * costmap_resolution_;
    const double dy = (y - center) * costmap_resolution_;
    const double radius = std::hypot(dx, dy);

    // Simple ripple pattern that changes over time and with cloud size.
    const double ripple = std::sin(radius + static_cast<double>(publish_count) * 0.5) +
                          0.0001 * static_cast<double>(last_cloud_size_);
    const int8_t value = ripple > 0.4 ? 60 : 0;
    costmap.data[i] = value;
  }

  return costmap;
}

QueryResponse EsdfMapCore::query(const QueryRequest& request) const {
  QueryResponse response;
  const double range_limit = request.max_range > 0.0 ? request.max_range : default_distance_;
  response.distance = computeMockDistance(request.position, range_limit);

  const double norm =
      std::sqrt(request.position.x * request.position.x + request.position.y * request.position.y +
                request.position.z * request.position.z);
  if (norm > 1e-6) {
    const double scale = -1.0 / norm;
    response.gradient.x = request.position.x * scale;
    response.gradient.y = request.position.y * scale;
    response.gradient.z = request.position.z * scale;
  }

  response.success = true;
  response.message = "Mock ESDF response. Replace with real distance field lookup.";
  return response;
}

double EsdfMapCore::computeMockDistance(const Vec3& position, double max_range) const {
  const double norm = std::sqrt(position.x * position.x + position.y * position.y +
                                position.z * position.z);
  return std::max(0.0, max_range - 0.1 * norm);
}

}  // namespace esdf_map
