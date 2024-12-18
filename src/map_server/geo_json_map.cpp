#include "map_server/geo_json_map.hpp"
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::map_server
{
    GeoJSONMap::GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node)
        : path_(path), node_(map_server_node)
    {
        if (path_.empty())
        {
            throw std::invalid_argument("GeoJSON file path cannot be empty");
        }

        from_ll_client_ = node_->create_client<robot_localization::srv::FromLL>("fromLL");
        while (!from_ll_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Interrupted while waiting for the robot localization /fromLL service. Exiting.");
                rclcpp::shutdown();
                rclcpp::sleep_for(std::chrono::seconds(10));
            }

            RCLCPP_INFO(node_->get_logger(), "service not available, waiting for the robot localization /fromLL...");
        }

        to_ll_client_ = node_->create_client<robot_localization::srv::ToLL>("toLL");
        while (!to_ll_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Interrupted while waiting for the robot localization /toLL service. Exiting.");
                rclcpp::shutdown();
                rclcpp::sleep_for(std::chrono::seconds(10));
            }

            RCLCPP_INFO(node_->get_logger(), "service not available, waiting for the robot localization /toLL...");
        }

        auto topic_name = node_->declare_parameter("foxglove_geojson_topic_name", "map/foxglove_geojson");
        foxglove_geo_json_publisher_ = node_->create_publisher<foxglove_msgs::msg::GeoJSON>(topic_name,
                                                          rclcpp::QoS(
                                                              rclcpp::KeepLast(1)).transient_local().reliable());

        // todo: requires refactor
        publishDatum();
    }

    json GeoJSONMap::pointToCoordinates(const geometry_msgs::msg::Point& point) const
    {
        const auto request = std::make_shared<robot_localization::srv::ToLL_Request>();
        request->map_point.x = point.x;
        request->map_point.y = point.y;
        auto result = to_ll_client_->async_send_request(request);
        if (const auto return_code =  rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(10)); return_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            const auto error_msg = "Failed to convert map points to LL: " + rclcpp::to_string(return_code);
            throw std::runtime_error(error_msg);
        }

        auto v = *result.get();
        json p = json::array();
        p.push_back(v.ll_point.longitude);
        p.push_back(v.ll_point.latitude);

        return p;
    }

    json GeoJSONMap::pointToCoordinates(const geometry_msgs::msg::Point32 &point) const {
        auto request = std::make_shared<robot_localization::srv::ToLL_Request>();
        request->map_point.x = point.x;
        request->map_point.y = point.y;
        auto result = to_ll_client_->async_send_request(request);
        if (const auto return_code =  rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(10)); return_code != rclcpp::FutureReturnCode::SUCCESS)
        {
            const auto error_msg = "Failed to convert map points to LL: " + rclcpp::to_string(return_code);
            throw std::runtime_error(error_msg);
        }

        auto v = *result.get();
        json p = json::array();
        p.push_back(v.ll_point.longitude);
        p.push_back(v.ll_point.latitude);

        return p;
    }

    json GeoJSONMap::mapAreaToGeoJSONFeature(const msg::Area& area) const {
        auto colorByType = [](const std::string& type) -> std::string {
            if (type == "navigation")
            {
                return "#0000ff";
            }
            if (type == "operation")
            {
                return "#00ff00";
            }
            if (type == "exclusion")
            {
                return "#ff0000";
            }
            return "#000000";
        };

        json feature;
        feature["type"] = "Feature";
        feature["properties"]["id"] = area.id;
        feature["properties"]["name"] = area.name;
        feature["properties"]["type"] = area.type == msg::Area::TYPE_NAVIGATION
                                            ? "navigation"
                                            : (
                                                area.type == msg::Area::TYPE_OPERATION ? "operation" : "exclusion");
        feature["properties"]["fill"] = colorByType(feature["properties"]["type"].get<std::string>());
        feature["properties"]["style"]["color"] = feature["properties"]["fill"];

        feature["geometry"]["type"] = "Polygon";
        feature["geometry"]["coordinates"] = json::array();

        json coordinates = json::array();
        for (const auto& point : area.area.polygon.points)
        {
            json p = pointToCoordinates(point);
            coordinates.push_back(p);
        }
        feature["geometry"]["coordinates"].push_back(coordinates);

        return feature;
    }

    geometry_msgs::msg::Point GeoJSONMap::movePointTowardsOrientation(const geometry_msgs::msg::Point& point,
                                                                      const geometry_msgs::msg::Quaternion& quaternion,
                                                                      double x) const
    {
        tf2::Quaternion tf2Quaternion;
        tf2::fromMsg(quaternion, tf2Quaternion);
        tf2::Matrix3x3 mat(tf2Quaternion);

        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        geometry_msgs::msg::Point outputPoint;

        outputPoint.x = point.x + x * std::cos(yaw);
        outputPoint.y = point.y + x * std::sin(yaw);
        outputPoint.z = point.z;

        return outputPoint;
    }

    json GeoJSONMap::dockingStationToGeoJSONFeature(const msg::DockingStation& docking_station)
    {
        json feature;
        feature["type"] = "Feature";
        feature["properties"]["id"] = docking_station.id;
        feature["properties"]["name"] = docking_station.name;
        feature["properties"]["type"] = "docking_station";
        feature["geometry"]["type"] = "LineString";

        auto coordinates = json::array();

        // calculate orientation point
        auto orientationPoint = movePointTowardsOrientation(docking_station.pose.pose.position,
                                                            docking_station.pose.pose.orientation, 0.5);

        coordinates.push_back(pointToCoordinates(docking_station.pose.pose.position));
        coordinates.push_back(pointToCoordinates(orientationPoint));

        feature["geometry"]["coordinates"] = coordinates;

        return feature;
    }

    void GeoJSONMap::eventuallyPublishFoxgloveGeoJSON(json data) const {
        if (foxglove_geo_json_publisher_ == nullptr)
        {
            RCLCPP_INFO(node_->get_logger(), "Foxglove GeoJSON publisher not initialized, skipping");
            return;
        }

        foxglove_msgs::msg::GeoJSON msg;
        msg.geojson = data.dump();

        foxglove_geo_json_publisher_->publish(msg);
    }

    msg::Map GeoJSONMap::load()
    {
        std::ifstream f(path_, std::ios::in);
        if (!f.is_open())
        {
            throw std::runtime_error("Could not open GeoJSON file: " + path_);
        }

        json data = json::parse(f);

        if (data["type"] != "FeatureCollection")
        {
            throw std::runtime_error("GeoJSON file is not a FeatureCollection type");
        }

        msg::Map map;
        map.header.stamp = node_->now(); // or use the modification time of the file?
        map.header.frame_id = node_->get_parameter("world_frame").as_string();

        for (const auto& feature : data["features"])
        {
            if (feature["type"] != "Feature")
            {
                RCLCPP_WARN(node_->get_logger(), "Non-feature object found in GeoJSON file");
                continue;
            }

            if (feature["geometry"]["type"] == "Polygon")
            {
                parsePolygonFeature(map, feature);
                continue;
            }

            if (feature["geometry"]["type"] == "LineString")
            {
                parseLineStringFeature(map, feature);
                continue;
            }

            RCLCPP_WARN(node_->get_logger(), "Unsupported geometry type: %s",
                        feature["geometry"]["type"].get<std::string>().c_str());
        }

        eventuallyPublishFoxgloveGeoJSON(data);

        return map;
    }

    void GeoJSONMap::save(msg::Map map)
    {
        json data;
        data["type"] = "FeatureCollection";
        data["features"] = json::array();

        for (const auto& area : map.areas)
        {
            auto feature = mapAreaToGeoJSONFeature(area);
            data["features"].push_back(feature);
        }

        for (const auto& docking_station : map.docking_stations)
        {
            auto feature = dockingStationToGeoJSONFeature(docking_station);
            data["features"].push_back(feature);
        }

        eventuallyPublishFoxgloveGeoJSON(data);
    }

    void GeoJSONMap::parsePolygonFeature(msg::Map& map, const json& feature)
    {
        msg::Area area;
        area.id = feature["properties"].value("id", "");
        area.name = feature["properties"].value("name", "");

        area.type = feature["properties"]["type"] == "navigation"
                        ? msg::Area::TYPE_NAVIGATION
                        : (
                            feature["properties"]["type"] == "operation" ? msg::Area::TYPE_OPERATION : msg::Area::TYPE_EXCLUSION);

        for (const auto& ll : feature["geometry"]["coordinates"][0])
        {
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
        if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(10)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // fetch result details and throw exception

        }

        auto v = *result.get();
        p.x = v.map_point.x;
        p.y = v.map_point.y;

        return p;
    }

    geometry_msgs::msg::PoseStamped GeoJSONMap::calculateTwoPointsPose(geometry_msgs::msg::Point32 origin,
                                                                       geometry_msgs::msg::Point32 point32)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = origin.x;
        pose.pose.position.y = origin.y;

        const double yaw = atan2(point32.y - origin.y, point32.x - origin.x);
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

        return pose;
    }

    void GeoJSONMap::parseLineStringFeature(msg::Map& map, const json& feature)
    {
        if (feature["properties"]["type"] != "docking_station")
        {
            RCLCPP_WARN(node_->get_logger(), "Unsupported line string type: %s",
                        feature["properties"]["type"].get<std::string>().c_str());
            return;
        }

        msg::DockingStation docking_station;
        docking_station.id = feature["properties"]["id"];
        docking_station.name = feature["properties"]["name"];

        // docking station line string must have two coordinates
        // the first one is the origin of the docking station (usually a middle of the charging connectors)
        // the second point is used to determine the orientation of the docking station (towards the robot's connectors)

        if (feature["geometry"]["coordinates"].size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Docking station line string must have at least two coordinates");
            return;
        }

        if (feature["geometry"]["coordinates"][0].size() > 2)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Docking station expected to have two coordinates, but has %d. Reading only the first two.",
                        static_cast<int>(feature["geometry"]["coordinates"][0].size()));
        }

        const geometry_msgs::msg::Point32 origin = parsePoint(feature["geometry"]["coordinates"][0]);
        const geometry_msgs::msg::Point32 orientation = parsePoint(feature["geometry"]["coordinates"][1]);

        docking_station.pose = calculateTwoPointsPose(origin, orientation);

        map.docking_stations.push_back(docking_station);
    }

    void GeoJSONMap::publishDatum() {
        datum_geopoint_publisher_ = node_->create_publisher<geographic_msgs::msg::GeoPoint>("map/datum",
                                                          rclcpp::QoS(
                                                              rclcpp::KeepLast(1)).transient_local().reliable());

        // send a 0,0 point to the robot localization to get the datum
        auto request = std::make_shared<robot_localization::srv::ToLL_Request>();
        auto result = to_ll_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(10)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            throw std::runtime_error("Could not retrieve datum point from robot localization");
        }
        datum_geopoint_publisher_->publish(result.get()->ll_point);
    }
} // OpenMowerMapServer
