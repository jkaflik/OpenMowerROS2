#pragma once

#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <nlohmann/json.hpp>
#include <foxglove_msgs/msg/geo_json.hpp>

#include "map_server_node.hpp"


using json = nlohmann::json;

namespace open_mower_next::map_server {
    class GeoJSONMap : public MapIO {
    public:
        explicit GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node);
        void save(msg::Map map);
        msg::Map load();

    private:
        void parsePolygonFeature(msg::Map &map, const json &feature);
        geometry_msgs::msg::Point32 parsePoint(json::const_reference value) const;
        geometry_msgs::msg::PoseStamped calculateTwoPointsPose(geometry_msgs::msg::Point32 origin, geometry_msgs::msg::Point32 point32);
        void parseLineStringFeature(msg::Map &map, const json &feature);
        json pointToCoordinates(const geometry_msgs::msg::Point& point) const;
        json pointToCoordinates(const geometry_msgs::msg::Point32& point) const;
        json mapAreaToGeoJSONFeature(const msg::Area& area);
        geometry_msgs::msg::Point movePointTowardsOrientation(const geometry_msgs::msg::Point& point,
                                                              const geometry_msgs::msg::Quaternion& quaternion,
                                                              double x) const;
        json dockingStationToGeoJSONFeature(const msg::DockingStation& docking_station);

        MapServerNode::SharedPtr node_;
        rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;
        rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_client_;

        void eventuallyPublishFoxgloveGeoJSON(json data);
        rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr foxglove_geo_json_publisher_;

        std::string path_;
    };

} // OpenMowerMapServer
