#include "coverage_server_node.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::coverage_server
{

CoverageServerNode::CoverageServerNode(const rclcpp::NodeOptions & options)
: Node("coverage_server", options)
{
  RCLCPP_INFO(get_logger(), "Starting Coverage Server node");

  robot_width_ = declare_parameter("robot_width", 0.325);
  operation_width_ = declare_parameter("operation_width", 0.065);
  headland_loops_ = declare_parameter("headland_loops", 3);
  min_turning_radius_ = declare_parameter("min_turning_radius", 0.01);
  swath_angle_ = declare_parameter("default_swath_angle", 0.0);
  spiral_order_size_ = declare_parameter("spiral_order_size", 1);

  RCLCPP_INFO(
    get_logger(), "Configured with robot_width=%.3f, mowing_tool_width=%.3f, headland_loops=%d",
    robot_width_, operation_width_, headland_loops_);

  // Setup services
  polygon_coverage_service_ = create_service<open_mower_next::srv::PolygonCoverage>(
    "polygon_coverage", std::bind(
                          &CoverageServerNode::handlePolygonCoverageRequest, this,
                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  area_coverage_service_ = create_service<open_mower_next::srv::AreaCoverage>(
    "area_coverage", std::bind(
                       &CoverageServerNode::handleAreaCoverageRequest, this, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3));

  // Setup map subscription
  map_subscription_ = create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&CoverageServerNode::mapCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "coverage/path", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "coverage/visualization", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

  RCLCPP_INFO(get_logger(), "Coverage Server initialized and ready");
}

CoverageServerNode::~CoverageServerNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down Coverage Server node");
}

void CoverageServerNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  current_map_ = *msg;
  RCLCPP_INFO(get_logger(), "Received updated map with %zu areas", msg->areas.size());
}

bool CoverageServerNode::isValidPolygon(const geometry_msgs::msg::PolygonStamped & polygon)
{
  // Check if the polygon has at least 3 points
  if (polygon.polygon.points.size() < 3) {
    RCLCPP_ERROR(get_logger(), "Invalid polygon: needs at least 3 points");
    return false;
  }

  return true;
}

// Service handlers (to be implemented)
void CoverageServerNode::handlePolygonCoverageRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_mower_next::srv::PolygonCoverage::Request> request,
  std::shared_ptr<open_mower_next::srv::PolygonCoverage::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received polygon coverage request");

  if (!isValidPolygon(request->polygon)) {
    response->code = open_mower_next::srv::PolygonCoverage::Response::CODE_INVALID_POLYGON;
    response->message = "The provided polygon is not valid";
    return;
  }

  std::vector<geometry_msgs::msg::PolygonStamped> exclusion_polygons;
  if (request->with_exclusions) {
    findExclusionsInPolygon(request->polygon, exclusion_polygons);
  }

  std::vector<nav_msgs::msg::Path> paths;
  std::string message;
  response->code = generateCoveragePath(request->polygon, exclusion_polygons, paths, message);
  response->message = message;

  if (response->code == open_mower_next::srv::PolygonCoverage::Response::CODE_SUCCESS) {
    response->paths = paths;
    response->polygon = request->polygon;
    response->area_ids = findAreasInPolygon(request->polygon);

    auto markers = createVisualizationMarkers(paths, request->polygon, exclusion_polygons);
    visualization_pub_->publish(markers);

    RCLCPP_INFO(get_logger(), "Generated %zu paths for polygon coverage", paths.size());
  }
}

void CoverageServerNode::handleAreaCoverageRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
  std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response)
{
  if (request->area_id.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area ID cannot be empty";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received area coverage request for area ID: %s", request->area_id.c_str());

  const auto area = findAreaById(request->area_id);

  if (area == nullptr) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area not found";
    return;
  }

  if (area.get()->type != open_mower_next::msg::Area::TYPE_OPERATION) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area is not of type OPERATION";
    return;
  }

  const auto & area_polygon = area.get()->area;

  if (area_polygon.polygon.points.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area has an invalid polygon";
    return;
  }

  // Find exclusions if requested
  std::vector<geometry_msgs::msg::PolygonStamped> exclusion_polygons;
  if (request->with_exclusions) {
    findExclusionsInPolygon(area_polygon, exclusion_polygons);
  }

  std::vector<nav_msgs::msg::Path> paths;
  std::string message;
  response->code = generateCoveragePath(area_polygon, exclusion_polygons, paths, message);
  response->message = message;

  if (response->code == open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS) {
    response->paths = paths;
    response->polygon = area_polygon;
    response->area_id = request->area_id;

    // Publish visualization
    auto markers = createVisualizationMarkers(paths, area_polygon, exclusion_polygons);
    visualization_pub_->publish(markers);

    RCLCPP_INFO(
      get_logger(), "Generated %zu paths for area ID: %s", paths.size(), request->area_id.c_str());
  }
}

msg::Area::SharedPtr CoverageServerNode::findAreaById(const std::string & area_id)
{
  geometry_msgs::msg::PolygonStamped result;
  result.header.frame_id = "map";
  result.header.stamp = now();

  for (const auto & area : current_map_.areas) {
    if (area.id == area_id) {
      RCLCPP_INFO(
        get_logger(), "Found area with ID: %s, name: %s", area_id.c_str(), area.name.c_str());

      return std::make_shared<msg::Area>(area);
    }
  }

  return nullptr;
}

std::vector<std::string> CoverageServerNode::findAreasInPolygon(
  const geometry_msgs::msg::PolygonStamped & polygon)
{
  std::vector<std::string> area_ids;

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_OPERATION) {
      // TODO: Implement proper polygon intersection test
      area_ids.push_back(area.id);
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu operation areas within the polygon", area_ids.size());
  return area_ids;
}

void CoverageServerNode::findExclusionsInPolygon(
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  exclusion_polygons.clear();

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_EXCLUSION) {
      // TODO: Implement proper polygon intersection test.
      // There are three scenarios:
      // 1. The exclusion area is completely inside the field polygon. Just add it.
      // 2. The exclusion area is completely outside the field polygon. Ignore it.
      // 3. The exclusion area intersects with the field polygon. Cut the intersection from the
      // field polygon.

      geometry_msgs::msg::PolygonStamped exclusion;
      exclusion.header = field_polygon.header;
      exclusion.polygon = area.area.polygon;

      exclusion_polygons.push_back(exclusion);
      RCLCPP_INFO(
        get_logger(), "Adding exclusion area with ID: %s, name: %s", area.id.c_str(),
        area.name.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu exclusion areas", exclusion_polygons.size());
}

f2c::types::Cells CoverageServerNode::convertToF2CCells(
  const geometry_msgs::msg::PolygonStamped & ros_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  f2c::types::Cell cell;
  f2c::types::LinearRing boundary;
  for (const auto & point : ros_polygon.polygon.points) {
    boundary.addPoint(f2c::types::Point(point.x, point.y));
  }

  auto first_point = boundary.at(0);
  auto last_point = boundary.at(boundary.size() - 1);

  // Add the first point again to close the loop if not already closed
  if (
    !boundary.isEmpty() &&
    (first_point.getX() != last_point.getX() || first_point.getY() != last_point.getY())) {
    boundary.addPoint(first_point);
  }

  cell.addGeometry(boundary);

  for (const auto & exclusion : exclusion_polygons) {
    f2c::types::LinearRing exclusion_ring;
    for (const auto & point : exclusion.polygon.points) {
      exclusion_ring.addPoint(f2c::types::Point(point.x, point.y));
    }

    auto first_point = boundary.at(0);
    auto last_point = boundary.at(boundary.size() - 1);

    if (
      !exclusion_ring.isEmpty() &&
      (first_point.getX() != last_point.getX() || first_point.getY() != last_point.getY())) {
      exclusion_ring.addPoint(first_point);
    }

    cell.addGeometry(exclusion_ring);
  }

  return f2c::types::Cells{cell};
}

nav_msgs::msg::Path CoverageServerNode::convertToRosPath(
  const f2c::types::Swaths & swaths, const std::string & frame_id)
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = frame_id;
  ros_path.header.stamp = now();

  for (const auto & swath : swaths) {
    auto path = swath.getPath();

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = ros_path.header;
      pose.pose.position.x = point.getX();
      pose.pose.position.y = point.getY();
      tf2::Quaternion q;
      q.setRPY(0, 0, point.getAngleFromPoint());
      pose.pose.orientation = tf2::toMsg(q);
      ros_path.poses.push_back(pose);
    }
  }

  return ros_path;
}

std::vector<nav_msgs::msg::Path> CoverageServerNode::convertToRosPath(
  const std::vector<f2c::types::Cells> & cells_vector, const std::string & frame_id)
{
  std::vector<nav_msgs::msg::Path> ros_paths;

  for (const auto & cells : cells_vector) {
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = frame_id;
    ros_path.header.stamp = now();

    for (const auto & linear_ring : cells.getGeometry(0)) {
      for (const auto & point : linear_ring) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = point.getX();
        pose.pose.position.y = point.getY();
        tf2::Quaternion q;
        q.setRPY(0, 0, point.getAngleFromPoint());
        pose.pose.orientation = tf2::toMsg(q);
        ros_path.poses.push_back(pose);
      }
    }

    ros_paths.push_back(ros_path);
  }
  return ros_paths;
}

uint16_t CoverageServerNode::generateCoveragePath(
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
  std::vector<nav_msgs::msg::Path> & response_paths, std::string & message)
{
  try {
    f2c::hg::ConstHL hg;

    RCLCPP_INFO(get_logger(), "Generating coverage path...");

    auto cells = convertToF2CCells(field_polygon, exclusion_polygons);

    f2c::types::Robot robot(robot_width_, operation_width_, min_turning_radius_);

    auto no_hl = cells;

    if (headland_loops_ > 0) {
      auto headland_width = robot.getCovWidth() * headland_loops_;

      RCLCPP_INFO(
        get_logger(), "Generating headland with width=%.2f (%d loops of %.2f)", headland_width,
        headland_loops_, robot.getCovWidth());

      no_hl = hg.generateHeadlands(cells, headland_width);
    }

    // Set the swath angle based on the configuration
    double swath_angle_rad = swath_angle_ * M_PI / 180.0;  // Convert degrees to radians

    f2c::sg::BruteForce swath_gen;

    const f2c::types::Swaths swaths =
      swath_gen.generateSwaths(swath_angle_rad, robot.getCovWidth(), no_hl.getGeometry(0));

    RCLCPP_INFO(get_logger(), "Generated %zu swaths", swaths.size());

    const f2c::rp::SpiralOrder spiral_sorter(spiral_order_size_);
    auto sorted_swaths = spiral_sorter.genSortedSwaths(swaths);

    RCLCPP_INFO(get_logger(), "Sorted swaths");

    response_paths.push_back(convertToRosPath(sorted_swaths, field_polygon.header.frame_id));

    // Add path for the headland if it exists
    if (headland_loops_ > 0) {
      auto cells_vector =
        hg.generateHeadlandSwaths(cells, operation_width_, headland_loops_, false);
      auto paths = convertToRosPath(cells_vector, field_polygon.header.frame_id);

      RCLCPP_INFO(get_logger(), "Generated %zu headland paths", paths.size());

      response_paths.insert(response_paths.end(), paths.begin(), paths.end());
    }

    nav_msgs::msg::Path merged_path;
    merged_path.header = field_polygon.header;
    for (const auto & path : response_paths) {
      merged_path.poses.insert(merged_path.poses.end(), path.poses.begin(), path.poses.end());
    }
    path_pub_->publish(merged_path);

    message = "Coverage path generated successfully";

    return open_mower_next::srv::PolygonCoverage::Response::CODE_SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error generating coverage path: %s", e.what());
    message = std::string("Error generating coverage path: ") + e.what();
    return open_mower_next::srv::PolygonCoverage::Response::CODE_UNKNOWN_ERROR;
  }
}

visualization_msgs::msg::MarkerArray CoverageServerNode::createVisualizationMarkers(
  const std::vector<nav_msgs::msg::Path> & paths,
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  visualization_msgs::msg::MarkerArray markers;

  // Create marker for the field boundary
  visualization_msgs::msg::Marker field_marker;
  field_marker.header = field_polygon.header;
  field_marker.ns = "area_boundary";
  field_marker.id = 0;
  field_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  field_marker.action = visualization_msgs::msg::Marker::ADD;
  field_marker.pose.orientation.w = 1.0;
  field_marker.scale.x = 0.1;  // Line width
  field_marker.color.r = 0.0;
  field_marker.color.g = 1.0;  // Green
  field_marker.color.b = 0.0;
  field_marker.color.a = 1.0;

  for (const auto & point : field_polygon.polygon.points) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = 0.05;  // Slight elevation above ground
    field_marker.points.push_back(ros_point);
  }

  // Close the loop
  if (!field_polygon.polygon.points.empty()) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = field_polygon.polygon.points[0].x;
    ros_point.y = field_polygon.polygon.points[0].y;
    ros_point.z = 0.05;
    field_marker.points.push_back(ros_point);
  }

  markers.markers.push_back(field_marker);

  // Create markers for exclusion areas
  for (size_t i = 0; i < exclusion_polygons.size(); i++) {
    visualization_msgs::msg::Marker exclusion_marker;
    exclusion_marker.header = field_polygon.header;
    exclusion_marker.ns = "exclusion_areas";
    exclusion_marker.id = i;
    exclusion_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    exclusion_marker.action = visualization_msgs::msg::Marker::ADD;
    exclusion_marker.pose.orientation.w = 1.0;
    exclusion_marker.scale.x = 0.1;  // Line width
    exclusion_marker.color.r = 1.0;  // Red
    exclusion_marker.color.g = 0.0;
    exclusion_marker.color.b = 0.0;
    exclusion_marker.color.a = 1.0;

    for (const auto & point : exclusion_polygons[i].polygon.points) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = 0.05;  // Slight elevation above ground
      exclusion_marker.points.push_back(ros_point);
    }

    // Close the loop
    if (!exclusion_polygons[i].polygon.points.empty()) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = exclusion_polygons[i].polygon.points[0].x;
      ros_point.y = exclusion_polygons[i].polygon.points[0].y;
      ros_point.z = 0.05;
      exclusion_marker.points.push_back(ros_point);
    }

    markers.markers.push_back(exclusion_marker);
  }

  // Create markers for the coverage paths
  for (size_t i = 0; i < paths.size(); i++) {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header = field_polygon.header;
    path_marker.ns = "coverage_paths";
    path_marker.id = i;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.05;  // Line width

    // Different color for each path
    if (i == 0) {  // Main path
      path_marker.color.r = 0.0;
      path_marker.color.g = 0.0;
      path_marker.color.b = 1.0;  // Blue
    } else {                      // Headland or additional paths
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.5;
      path_marker.color.b = 0.0;  // Orange
    }
    path_marker.color.a = 1.0;

    for (const auto & pose : paths[i].poses) {
      path_marker.points.push_back(pose.pose.position);
    }

    markers.markers.push_back(path_marker);

    // Add direction arrows at regular intervals along the path
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header = field_polygon.header;
    arrow_marker.ns = "path_directions";
    arrow_marker.id = i;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.scale.x = 0.3;  // Arrow length
    arrow_marker.scale.y = 0.1;  // Arrow width
    arrow_marker.scale.z = 0.1;  // Arrow height
    arrow_marker.color = path_marker.color;

    // Add direction arrows at regular intervals
    const int interval = std::max(1, static_cast<int>(paths[i].poses.size() / 10));
    for (size_t j = 0; j < paths[i].poses.size(); j += interval) {
      arrow_marker.id = i * 1000 + j;  // Unique ID for each arrow
      arrow_marker.pose = paths[i].poses[j].pose;
      markers.markers.push_back(arrow_marker);
    }
  }

  RCLCPP_INFO(get_logger(), "Created %zu visualization markers", markers.markers.size());
  return markers;
}

}  // namespace open_mower_next::coverage_server