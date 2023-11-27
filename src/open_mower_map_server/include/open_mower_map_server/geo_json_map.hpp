#pragma once

#include <robot_localization/srv/from_ll.hpp>
#include <nlohmann/json.hpp>

#include "map_server_node.hpp"


using json = nlohmann::json;

namespace open_mower_map_server {
    class GeoJSONMap : public MapIO {
    public:
        explicit GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node);

        void save(open_mower_map_server::msg::Map map);
        open_mower_map_server::msg::Map load();

    private:
        void parsePolygonFeature(open_mower_map_server::msg::Map &map, const json &feature);
        geometry_msgs::msg::Point32 parsePoint(json::const_reference value) const;
        static static geometry_msgs::msg::PoseStamped calculateTwoPointsPose(geometry_msgs::msg::Point32 origin, geometry_msgs::msg::Point32 point32);
        void parseLineStringFeature(open_mower_map_server::msg::Map &map, const json &feature);

        MapServerNode::SharedPtr node_;
        rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;

        std::string path_;
    };

} // OpenMowerMapServer
