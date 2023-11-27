#include "open_mower_map_server/map_server_node.hpp"
#include "open_mower_map_server/polygon_iterator.hpp"
#include "open_mower_map_server/geo_json_map.hpp"

namespace open_mower_map_server {
    MapServerNode::MapServerNode(const rclcpp::NodeOptions &options) : rclcpp::Node("map_server_node", options) {
        configureMap();
        configureGaussianBlur();

        auto topic_name = declare_parameter("topic_name", "map");
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name,
                                                                        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        current_map_ = map_io_->load();
        map_publisher_->publish(mapToOccupancyGrid(current_map_));
    }

    void MapServerNode::configureMap() {
        map_type_ = declare_parameter("type", "geojson");
        map_file_ = declare_parameter("path", "~/.openmower/map.geojson");

        if (map_type_ == "geojson") {
            map_io_ = new GeoJSONMap(map_file_, std::shared_ptr<MapServerNode>(this, [](MapServerNode *) {}));
        } else {
            throw std::runtime_error("Unsupported map type: " + map_type_);
        }
    }

    void MapServerNode::configureGaussianBlur() {
        use_gaussian_blur_ = declare_parameter("grid.use_gaussian_blur", false);
        if (use_gaussian_blur_) {
            RCLCPP_INFO(get_logger(), "Using Gaussian blur");

            std::vector<double> kernel = declare_parameter("map.gaussian_blur.kernel", std::vector<double>{1, 2, 1, 2, 4, 2, 1, 2, 1});
            std::vector<float> kernel_float(kernel.begin(), kernel.end());

            gaussian_filter_ = new SomeGaussianFilter(kernel_float);
        }
    }

    nav_msgs::msg::OccupancyGrid MapServerNode::mapToOccupancyGrid(open_mower_map_server::msg::Map map) {
        float minX, minY, maxX, maxY;

        for (const auto &area : map.areas) {
            for (const auto &point : area.area.polygon.points) {
                minX = std::min(minX, point.x);
                minY = std::min(minY, point.y);
                maxX = std::max(maxX, point.x);
                maxY = std::max(maxY, point.y);
            }
        }

        maxX += 1;
        maxY += 1;
        minX -= 1;
        minY -= 1;

        nav_msgs::msg::OccupancyGrid occupancy_grid;
        occupancy_grid.header = map.header;
        occupancy_grid.info.origin.position.x = (maxX + minX) / 2;
        occupancy_grid.info.origin.position.y = (maxY + minY) / 2;
        occupancy_grid.info.resolution = 0.1;
        occupancy_grid.info.width = (maxX - minX);
        occupancy_grid.info.height = (maxY - minY);
        occupancy_grid.info.map_load_time = this->now();

        for (const auto &area : map.areas) {
            uint8_t value = area.type == msg::Area::TYPE_EXCLUSION ? 100 : 0;

            fillGridWithPolygon(occupancy_grid, area.area.polygon, value);
        }

        RCLCPP_INFO(get_logger(), "Map size: %d x %d (%.2fm resolution)", occupancy_grid.info.width, occupancy_grid.info.height, occupancy_grid.info.resolution);

        if (gaussian_filter_) {
            RCLCPP_INFO(get_logger(), "Applying Gaussian filter");

            gaussian_filter_->apply(occupancy_grid.data, occupancy_grid.info.width, occupancy_grid.info.height);
        }

        return occupancy_grid;
    }

    void MapServerNode::fillGridWithPolygon(nav_msgs::msg::OccupancyGrid &occupancy_grid, const geometry_msgs::msg::Polygon &polygon, uint8_t value) {
        auto iterator = PolygonGridIterator(polygon, occupancy_grid.info.resolution);
        while (iterator.next()) {
            auto point = *iterator;
            auto x = point.x - occupancy_grid.info.origin.position.x;
            auto y = point.y - occupancy_grid.info.origin.position.y;
            auto index = (int) (x / occupancy_grid.info.resolution) + (int) (y / occupancy_grid.info.resolution) * occupancy_grid.info.width;

            if (index >= occupancy_grid.data.size()) {
                RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d Grid size: %d", index, occupancy_grid.data.size());

                continue;
            }

            occupancy_grid.data[index] = value;
        }
    }
}
