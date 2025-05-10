#pragma once

#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/srv/area_coverage.hpp"
#include "open_mower_next/srv/polygon_coverage.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fields2cover.h>

namespace open_mower_next::coverage_server
{

class CoverageServerNode : public rclcpp::Node
{
public:
  explicit CoverageServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CoverageServerNode();

private:
  double robot_width_;         // Width of the robot platform
  double operation_width_;   // Width of the mowing tool/blade
  int headland_loops_;         // Number of loops to create in the headland
  double min_turning_radius_;  // Minimum turning radius of the robot
  double swath_angle_;         // Angle for swath patterns
  unsigned long spiral_order_size_; // Size of the spiral order for coverage

  rclcpp::Service<open_mower_next::srv::PolygonCoverage>::SharedPtr polygon_coverage_service_;
  rclcpp::Service<open_mower_next::srv::AreaCoverage>::SharedPtr area_coverage_service_;

  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_subscription_;
  open_mower_next::msg::Map current_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void handlePolygonCoverageRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_mower_next::srv::PolygonCoverage::Request> request,
    std::shared_ptr<open_mower_next::srv::PolygonCoverage::Response> response);

  void handleAreaCoverageRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
    std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response);

  void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);

  uint16_t generateCoveragePath(
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
    std::vector<nav_msgs::msg::Path> & response_paths, std::string & message);

  void findExclusionsInPolygon(
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons);

  msg::Area::SharedPtr findAreaById(const std::string & area_id);

  std::vector<std::string> findAreasInPolygon(
    const geometry_msgs::msg::PolygonStamped & polygon);

  f2c::types::Cells convertToF2CCells(
    const geometry_msgs::msg::PolygonStamped & ros_polygon,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons);

  nav_msgs::msg::Path convertToRosPath(
    const f2c::types::Swaths & swaths, const std::string & frame_id);
  std::vector<nav_msgs::msg::Path> convertToRosPath(
    const std::vector<f2c::types::Cells> & cells_vector, const std::string & frame_id);

  visualization_msgs::msg::MarkerArray createVisualizationMarkers(
    const std::vector<nav_msgs::msg::Path> & paths,
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons);

  bool isValidPolygon(const geometry_msgs::msg::PolygonStamped & polygon);
};

}  // namespace open_mower_next::coverage_server
