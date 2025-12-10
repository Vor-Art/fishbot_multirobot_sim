#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "esdf_map/esdf_map_core.hpp"
#include "esdf_map/srv/query_esdf.hpp"

namespace esdf_map
{
    class EsdfMapNode : public rclcpp::Node
    {
    public:
        explicit EsdfMapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        void handleCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void publishOutputs();

        nav_msgs::msg::OccupancyGrid buildMockCostmap(const rclcpp::Time &stamp) const;
        sensor_msgs::msg::PointCloud2 buildMockEsdfGrid(const rclcpp::Time &stamp) const;
        void handleQuery(const std::shared_ptr<srv::QueryEsdf::Request> request,
                         std::shared_ptr<srv::QueryEsdf::Response> response);
        nav_msgs::msg::OccupancyGrid toOccupancyMsg(const Costmap2d &costmap,
                                                    const rclcpp::Time &stamp) const;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
        rclcpp::Service<srv::QueryEsdf>::SharedPtr query_service_;
        rclcpp::TimerBase::SharedPtr publish_timer_;

        sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

        EsdfMapCore core_;

        std::string cloud_topic_;
        double publish_rate_hz_{};
        double costmap_resolution_{};
        double costmap_size_m_{};
        double default_distance_{};
        mutable std::size_t publish_count_{0};
    };

} // namespace esdf_map
