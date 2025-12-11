#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "esdf_map/esdf_map_core.hpp"
#include "esdf_map/srv/query_esdf.hpp"
#include "esdf_map/utils.hpp"

namespace esdf_map
{

    class EsdfMapNode : public rclcpp::Node {
    public:
        using PointCloud2 = sensor_msgs::msg::PointCloud2;
        using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
        using EsdfQuery = esdf_map::srv::QueryEsdf;

        explicit EsdfMapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        struct Bot {
            int id = 0;
            std::string name;
            std::string cloud_topic;
            std::string world_frame;  // botX/world
            std::string sensor_frame; // botX/lidar_link
            rclcpp::Subscription<PointCloud2>::SharedPtr sub;
        };

        // Parameters
        void declareAndLoadParams();
        void buildBotList();
        esdf_map::EsdfMapCore::Config makeCoreConfig() const;

        // Subscriptions / callbacks
        void setupSubscriptions();
        void handleCloud(int bot_id, const PointCloud2::SharedPtr msg);
        bool lookupLidarPose(const std::string &lidar_frame,
                             const rclcpp::Time &stamp,
                             Eigen::Isometry3d &T_M_L);

        // Timers
        void setupTimers();
        void esdfUpdateTimerCb();
        void publishTimerCb();

        // Publishers
        void publishGrid();
        void publishCostmap2D();

        // Service
        void setupService();
        void handleQuery(const std::shared_ptr<EsdfQuery::Request> request,
                         std::shared_ptr<EsdfQuery::Response> response);

        // Helpers
        void pointCloud2ToPcl(const PointCloud2 &msg, esdf_map::PointCloud &cloud_out) const;
        void fillCostmapMsg(const esdf_map::Slice2D &slice, OccupancyGrid &grid_msg) const;

        // Core map
        std::unique_ptr<EsdfMapCore> core_;

        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Robot inputs
        std::string bot_prefix_;
        std::vector<int64_t> bot_ids_;
        std::string bot_cloud_topic_;
        std::string bot_sensor_frame_;
        std::string world_frame_;
        double tf_timeout_sec_{0.1};

        std::map<int, Bot> bots_;

        // Map / integration params
        double esdf_resolution_{0.1};
        double map_size_x_{30.0};
        double map_size_y_{30.0};
        double map_size_z_{5.0};
        double map_origin_x_{-15.0};
        double map_origin_y_{-15.0};
        double map_origin_z_{0.0};

        double max_ray_length_{30.0};
        double truncation_distance_{0.3};
        double esdf_max_distance_{5.0};

        bool integrate_every_cloud_{true};
        double esdf_update_rate_hz_{2.0};

        // Output params
        double publish_rate_hz_{1.0};
        bool publish_full_grid_{true};
        bool publish_costmap_2d_{true};

        double costmap_layer_z_{0.5};
        double costmap_free_distance_{0.5};
        double costmap_lethal_distance_{0.1};

        // Publishers / timers / service
        rclcpp::Publisher<PointCloud2>::SharedPtr esdf_grid_pub_;
        rclcpp::Publisher<OccupancyGrid>::SharedPtr costmap_pub_;
        rclcpp::TimerBase::SharedPtr esdf_update_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Service<EsdfQuery>::SharedPtr query_srv_;
    };

} // namespace esdf_map
