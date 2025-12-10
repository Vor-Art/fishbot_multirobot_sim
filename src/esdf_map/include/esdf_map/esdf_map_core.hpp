#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace esdf_map {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Costmap2d {
  double resolution{0.25};
  double size_m{20.0};
  uint32_t width{0};
  uint32_t height{0};
  std::vector<int8_t> data;
};

struct QueryRequest {
  Vec3 position;
  double max_range{0.0};
};

struct QueryResponse {
  double distance{0.0};
  Vec3 gradient;
  bool success{false};
  std::string message;
};

class EsdfMapCore {
public:
  EsdfMapCore();
  EsdfMapCore(double default_distance, double costmap_resolution, double costmap_size_m);

  void setDefaultDistance(double default_distance);
  void setCostmap(double resolution, double size_m);

  void updateFromCloud(std::size_t point_count);

  Costmap2d buildCostmap(std::size_t publish_count) const;
  QueryResponse query(const QueryRequest& request) const;

private:
  double computeMockDistance(const Vec3& position, double max_range) const;

  double default_distance_{3.0};
  double costmap_resolution_{0.25};
  double costmap_size_m_{20.0};
  std::size_t last_cloud_size_{0};
};

}  // namespace esdf_map
