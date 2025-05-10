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
  min_turning_radius_ = declare_parameter("min_turning_radius", 0.01);

  RCLCPP_INFO(
    get_logger(), "Configured with robot_width=%.3f, mowing_tool_width=%.3f", robot_width_,
    operation_width_);

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
nav_msgs::msg::Path CoverageServerNode::convertToRosPath(
  const f2c::types::Path & f2c_path, const nav_msgs::msg::Path & ros_path)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  for (const auto & path_state : f2c_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;

    pose.pose.position.x = path_state.point.getX();
    pose.pose.position.y = path_state.point.getY();

    tf2::Quaternion q;
    q.setRPY(0, 0, path_state.angle);
    pose.pose.orientation = tf2::toMsg(q);

    poses.push_back(pose);
  }

  nav_msgs::msg::Path path;
        path.header = ros_path.header;
        path.poses = poses;

  return path;
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

  nav_msgs::msg::Path path;
  path.header.frame_id = area_polygon.header.frame_id;

  std::string message;
  response->code = generateCoveragePath(
    request->headland_loops, request->swath_angle, area_polygon, exclusion_polygons, path, message);
  response->message = message;

  if (response->code == open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS) {
    response->path = path;
    response->polygon = area_polygon;
    response->area_id = request->area_id;

    // Publish visualization
    auto markers = createVisualizationMarkers(path, area_polygon, exclusion_polygons);
    visualization_pub_->publish(markers);

    RCLCPP_INFO(
      get_logger(), "Generated path with a %zu poses for area ID: %s", path.poses.size(), request->area_id.c_str());
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

uint16_t CoverageServerNode::generateCoveragePath(
  const uint16_t headland_loops, const uint16_t swath_angle,
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
  nav_msgs::msg::Path & path, std::string & message)
{
  try {
    f2c::hg::ConstHL hg;

    RCLCPP_INFO(get_logger(), "Generating coverage path...");

    auto cells = convertToF2CCells(field_polygon, exclusion_polygons);

    f2c::types::Robot robot(robot_width_, operation_width_, min_turning_radius_);

    auto mainland = cells;

    if (headland_loops > 0) {
      auto headland_width = robot.getCovWidth() * headland_loops;

      RCLCPP_INFO(
        get_logger(), "Generating headland with width=%.2f (%d loops of %.2f)", headland_width,
        headland_loops, robot.getCovWidth());

      mainland = hg.generateHeadlands(cells, headland_width);
    }

    // Set the swath angle based on the configuration
    double swath_angle_rad = swath_angle * M_PI / 180.0;  // Convert degrees to radians

    f2c::sg::BruteForce swath_gen;

    f2c::obj::SwathLength swath_length;
    auto best_angle =
      swath_gen.computeBestAngle(swath_length, robot.getCovWidth(), mainland.getGeometry(0));
    auto computed_angle = best_angle + swath_angle_rad;

    const f2c::types::Swaths swaths =
      swath_gen.generateSwaths(computed_angle, robot.getCovWidth(), mainland.getGeometry(0));

    RCLCPP_INFO(get_logger(), "Generated %zu swaths", swaths.size());

    const f2c::rp::BoustrophedonOrder sorter;
    f2c::types::Swaths sorted_swaths = sorter.genSortedSwaths(swaths);

    RCLCPP_INFO(get_logger(), "Sorted swaths");

    // response_paths.push_back(convertToRosPath(sorted_swaths, field_polygon.header.frame_id));

    // Add path for the headland if it exists
    if (headland_loops > 0) {
      auto cells_vector = hg.generateHeadlandSwaths(cells, operation_width_, headland_loops, false);

      for (auto & cell : cells_vector) {
        for (auto & ring : cell.getGeometry(0)) {
          sorted_swaths.append(
            f2c::types::LineString(ring), robot.getCovWidth(), f2c::types::SwathType::HEADLAND);
        }
      }

      // auto paths = convertToRosPath(cells_vector, field_polygon.header.frame_id);

      // RCLCPP_INFO(get_logger(), "Generated %zu headland paths", paths.size());

      // response_paths.insert(response_paths.end(), paths.begin(), paths.end());
    }

    f2c::pp::PathPlanning path_planner;
    f2c::pp::DubinsCurves dubins;
    f2c::types::Path dubins_path = path_planner.planPath(robot, sorted_swaths, dubins);

    path = convertToRosPath(dubins_path, path);

    message = "Coverage path generated successfully";

    return open_mower_next::srv::PolygonCoverage::Response::CODE_SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error generating coverage path: %s", e.what());
    message = std::string("Error generating coverage path: ") + e.what();
    return open_mower_next::srv::PolygonCoverage::Response::CODE_UNKNOWN_ERROR;
  }
}

visualization_msgs::msg::MarkerArray CoverageServerNode::createVisualizationMarkers(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PolygonStamped & field_polygon,
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

  visualization_msgs::msg::Marker path_marker;
  path_marker.header = field_polygon.header;
  path_marker.ns = "coverage_path";
  path_marker.id = 1;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;  // Line width
  path_marker.color.r = 0.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 1.0;  // Blue
  path_marker.color.a = 1.0;

  for (const auto & pose : path.poses) {
    path_marker.points.push_back(pose.pose.position);
  }

  markers.markers.push_back(path_marker);

  // Add direction arrows at regular intervals along the path
  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header = field_polygon.header;
  arrow_marker.ns = "path_directions";
  arrow_marker.id = 1;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  arrow_marker.scale.x = 0.3;  // Arrow length
  arrow_marker.scale.y = 0.1;  // Arrow width
  arrow_marker.scale.z = 0.1;  // Arrow height
  arrow_marker.color = path_marker.color;

  // Add direction arrows at regular intervals
  const int interval = std::max(1, static_cast<int>(path.poses.size() / 10));
  for (size_t j = 0; j < path.poses.size(); j += interval) {
    arrow_marker.id = 1000 + j;  // Unique ID for each arrow
    arrow_marker.pose = path.poses[j].pose;
    markers.markers.push_back(arrow_marker);
  }

  RCLCPP_INFO(get_logger(), "Created %zu visualization markers", markers.markers.size());
  return markers;
}

}  // namespace open_mower_next::coverage_server