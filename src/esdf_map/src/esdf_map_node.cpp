#include "esdf_map/esdf_map_node.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <vector>

namespace esdf_map {
using namespace std::chrono_literals;

EsdfMapNode::EsdfMapNode(const rclcpp::NodeOptions& options) : Node("esdf_map_node", options) {
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/bot1/cloud_registered");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
  costmap_resolution_ = declare_parameter<double>("costmap_resolution", 0.25);
  costmap_size_m_ = declare_parameter<double>("costmap_size_m", 20.0);
  default_distance_ = declare_parameter<double>("default_distance", 3.0);

  costmap_resolution_ = std::max(0.01, costmap_resolution_);
  costmap_size_m_ = std::max(costmap_size_m_, 2.0 * costmap_resolution_);
  core_.setDefaultDistance(default_distance_);
  core_.setCostmap(costmap_resolution_, costmap_size_m_);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EsdfMapNode::handleCloud, this, std::placeholders::_1));

  grid_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("esdf/grid", rclcpp::QoS(10));
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("esdf/costmap_2d", rclcpp::QoS(1));
  query_service_ = create_service<srv::QueryEsdf>("esdf/query",
      std::bind(&EsdfMapNode::handleQuery, this, std::placeholders::_1, std::placeholders::_2));

  const std::chrono::duration<double> period =
      publish_rate_hz_ > 0.0 ? std::chrono::duration<double>(1.0 / publish_rate_hz_): std::chrono::duration<double>(0.5);
  publish_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EsdfMapNode::publishOutputs, this));

  RCLCPP_INFO(get_logger(), "ESDF map mock node ready. Listening to %s", cloud_topic_.c_str());
}

void EsdfMapNode::handleCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  last_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  last_cloud_->header.frame_id = "map_origin";
  core_.updateFromCloud(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));
  RCLCPP_DEBUG(get_logger(), "Received cloud with %u points", msg->width * msg->height);
}

void EsdfMapNode::publishOutputs() {
  const auto stamp = get_clock()->now();

  grid_pub_->publish(buildMockEsdfGrid(stamp));
  costmap_pub_->publish(buildMockCostmap(stamp));

  ++publish_count_;
}

sensor_msgs::msg::PointCloud2 EsdfMapNode::buildMockEsdfGrid(const rclcpp::Time& stamp) const {
  if (last_cloud_) {
    auto cloud = *last_cloud_;
    cloud.header.stamp = stamp;
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "map_origin";
  cloud.height = 1;
  cloud.width = 0;
  return cloud;
}

nav_msgs::msg::OccupancyGrid EsdfMapNode::buildMockCostmap(const rclcpp::Time& stamp) const {
  const auto costmap = core_.buildCostmap(publish_count_);
  return toOccupancyMsg(costmap, stamp);
}

void EsdfMapNode::handleQuery(
    const std::shared_ptr<srv::QueryEsdf::Request> request,
    std::shared_ptr<srv::QueryEsdf::Response> response) {
  const auto& p = request->position;

  QueryRequest core_request;
  core_request.position = { p.x, p.y, p.z };
  core_request.max_range = request->max_range;

  const auto result = core_.query(core_request);
  response->distance = result.distance;
  response->gradient.x = result.gradient.x;
  response->gradient.y = result.gradient.y;
  response->gradient.z = result.gradient.z;
  response->success = result.success;
  response->message = result.message;
}

nav_msgs::msg::OccupancyGrid EsdfMapNode::toOccupancyMsg(
    const Costmap2d& costmap, const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map_origin";

  msg.info.resolution = costmap.resolution;
  msg.info.width = costmap.width;
  msg.info.height = costmap.height;

  const double offset = -0.5 * static_cast<double>(costmap.width) * costmap.resolution;
  msg.info.origin.position.x = offset;
  msg.info.origin.position.y = offset;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = costmap.data;
  return msg;
}

}  // namespace esdf_map
