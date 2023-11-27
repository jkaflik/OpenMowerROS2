#include "open_mower_map_server/geo_json_map.hpp"
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace open_mower_map_server {
    GeoJSONMap::GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node)
            : path_(path), node_(map_server_node) {
        if (path_.empty()) {
            throw std::invalid_argument("GeoJSON file path cannot be empty");
        }

        from_ll_client_ = node_->create_client<robot_localization::srv::FromLL>("fromLL");

        while (!from_ll_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the robot localization /fromLL service. Exiting.");
                rclcpp::shutdown();
            }

            RCLCPP_INFO(node_->get_logger(), "service not available, waiting for the robot localization /fromLL...");
        }
    }

    msg::Map GeoJSONMap::load() {
        std::ifstream f(path_, std::ios::in);
        if (!f.is_open()) {
            throw std::runtime_error("Could not open GeoJSON file: " + path_);
        }

        json data = json::parse(f);

        if (data["type"] != "FeatureCollection") {
            throw std::runtime_error("GeoJSON file is not a FeatureCollection type");
        }

        msg::Map map;
        map.header.stamp = node_->now(); // or use the modification time of the file?
        map.header.frame_id = node_->get_parameter("world_frame").as_string();

        for (const auto &feature: data["features"]) {
            if (feature["type"] != "Feature") {
                RCLCPP_WARN(node_->get_logger(), "Non-feature object found in GeoJSON file");
                continue;
            }

            if (feature["geometry"]["type"] == "Polygon") {
                parsePolygonFeature(map, feature);
                continue;
            }

            if (feature["geometry"]["type"] == "LineString") {
                parseLineStringFeature(map, feature);
                continue;
            }

            RCLCPP_WARN(node_->get_logger(), "Unsupported geometry type: %s", feature["geometry"]["type"].get<std::string>().c_str());
        }
    }

    void GeoJSONMap::save(msg::Map map) {
        throw std::runtime_error("Not implemented");
    }

    void GeoJSONMap::parsePolygonFeature(msg::Map &map, const json &feature) {
        msg::Area area;
        area.id = feature["properties"].value("id", "");
        area.name = feature["properties"].value("name", "");

        area.type = feature["properties"]["type"] == "navigation" ? msg::Area::TYPE_NAVIGATION : (
                feature["properties"]["type"] == "lawn" ? msg::Area::TYPE_LAWN : msg::Area::TYPE_EXCLUSION);

        for (const auto &ll: feature["geometry"]["coordinates"][0]) {
            auto p = parsePoint(ll);
            area.area.polygon.points.push_back(p);
        }

        map.areas.push_back(area);
    }

    geometry_msgs::msg::Point32 GeoJSONMap::parsePoint(json::const_reference value) const
    {
        geometry_msgs::msg::Point32 p;

        auto request = std::make_shared<robot_localization::srv::FromLL_Request>();
        request->ll_point.latitude = value[1];
        request->ll_point.longitude = value[0];
        auto result = from_ll_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Could not convert LL to map coordinates");
        }

        auto v = *result.get();
        p.x = v.map_point.x;
        p.y = v.map_point.y;

        return p;
    }

    geometry_msgs::msg::PoseStamped GeoJSONMap::calculateTwoPointsPose(geometry_msgs::msg::Point32 origin,
        geometry_msgs::msg::Point32 point32) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = origin.x;
        pose.pose.position.y = origin.y;

        const double yaw = atan2(point32.y - origin.y, point32.x - origin.x);
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

        return pose;
    }

    void GeoJSONMap::parseLineStringFeature(msg::Map &map, const json &feature) {
        if (feature["properties"]["type"] != "docking_station") {
            RCLCPP_WARN(node_->get_logger(), "Unsupported line string type: %s", feature["properties"]["type"].get<std::string>().c_str());
            return;
        }

        msg::DockingStation docking_station;
        docking_station.id = feature["properties"]["id"];
        docking_station.name = feature["properties"]["name"];

        // docking station line string must have two coordinates
        // the first one is the origin of the docking station (usually a middle of the charging connectors)
        // the second point is used to determine the orientation of the docking station (towards the robot's connectors)

        if (feature["geometry"]["coordinates"].size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Docking station line string must have at least two coordinates");
            return;
        }

        if (feature["geometry"]["coordinates"][0].size() > 2) {
            RCLCPP_WARN(node_->get_logger(), "Docking station expected to have two coordinates, but has %d. Reading only the first two.", static_cast<int>(feature["geometry"]["coordinates"][0].size()));
        }

        const geometry_msgs::msg::Point32 origin = parsePoint(feature["geometry"]["coordinates"][0]);
        const geometry_msgs::msg::Point32 orientation = parsePoint(feature["geometry"]["coordinates"][1]);

        docking_station.pose = calculateTwoPointsPose(origin, orientation);

        map.docking_stations.push_back(docking_station);
    }
} // OpenMowerMapServer