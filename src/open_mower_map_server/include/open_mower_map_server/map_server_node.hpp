#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/nav_msgs/msg/occupancy_grid.hpp>

#include "open_mower_map_server/msg/map.hpp"
#include "open_mower_map_server/some_gaussian_filter.hpp"

namespace open_mower_map_server {
    class MapIO {
    public:
        virtual ~MapIO() = default;

        virtual void save(open_mower_map_server::msg::Map map) = 0;

        virtual open_mower_map_server::msg::Map load() = 0;
    };

    class MapServerNode final : public rclcpp::Node {
    public:
        explicit MapServerNode(const rclcpp::NodeOptions &options);

        ~MapServerNode() override = default;

    private:
        nav_msgs::msg::OccupancyGrid mapToOccupancyGrid(open_mower_map_server::msg::Map map);

        MapIO *map_io_;

        void fillGridWithPolygon(nav_msgs::msg::OccupancyGrid &occupancy_grid,
                                      const geometry_msgs::msg::Polygon &polygon,
                                      uint8_t value);

        // parameters
        std::string map_type_;
        std::string map_file_;
        bool use_gaussian_blur_;

        void configureGaussianBlur();

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

        void configureMap();

        SomeGaussianFilter *gaussian_filter_;
        open_mower_map_server::msg::Map current_map_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    };
}
