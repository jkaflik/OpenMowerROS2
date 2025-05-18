#pragma once

#include "fields2cover/types/Path.h"
#include "fields2cover/types/Swaths.h"

#include <tf2/LinearMath/Quaternion.hpp>

#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::coverage_server::utils
{
inline geometry_msgs::msg::Point toMsg(const f2c::types::Point & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.getX();
  msg.y = point.getY();
  return msg;
}

inline bool isValid(const geometry_msgs::msg::Polygon & polygon)
{
  if (polygon.points.size() < 3) {
    return false;
  }

  return true;
}

inline f2c::types::LinearRing toLinearRing(const geometry_msgs::msg::Polygon & polygon)
{
  f2c::types::LinearRing ring;
  for (const auto & point : polygon.points) {
    ring.addPoint(f2c::types::Point(point.x, point.y));
  }

  auto first_point = ring.at(0);
  auto last_point = ring.at(ring.size() - 1);

  // Add the first point again to close the loop if not already closed
  if (
    !ring.isEmpty() &&
    (first_point.getX() != last_point.getX() || first_point.getY() != last_point.getY())) {
    ring.addPoint(first_point);
  }

  return ring;
}

inline f2c::types::LinearRing toLinearRing(const geometry_msgs::msg::PolygonStamped & polygon)
{
  return toLinearRing(polygon.polygon);
}

inline f2c::types::Cell toCell(
  const geometry_msgs::msg::PolygonStamped & boundary_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  f2c::types::Cell cell;

  const auto boundary_ring = toLinearRing(boundary_polygon);
  cell.addRing(boundary_ring);

  for (const auto & exclusion : exclusion_polygons) {
    const auto exclusion_ring = toLinearRing(exclusion);
    cell.addRing(exclusion_ring);
  }

  return cell;
}

inline geometry_msgs::msg::PoseStamped toMsg(
  const double x, const double y, const double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

inline nav_msgs::msg::Path toMsg(const f2c::types::Swaths & swaths, const std::string & frame_id)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;

  for (const auto & swath : swaths) {
    for (int j = 0; j < swath.numPoints(); ++j) {
      const auto & point = swath.getPoint(j);
      const geometry_msgs::msg::PoseStamped pose =
        toMsg(point.getX(), point.getY(), point.getAngleFromPoint(), frame_id);
      msg.poses.push_back(pose);
    }
  }

  return msg;
}

inline nav_msgs::msg::Path toMsg(const f2c::types::Path & path, const std::string & frame_id)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;

  for (const auto & state : path) {
    const geometry_msgs::msg::PoseStamped pose =
      toMsg(state.point.getX(), state.point.getY(), state.angle, frame_id);
    msg.poses.push_back(pose);
  }

  return msg;
}
}  // namespace open_mower_next::coverage_server::utils