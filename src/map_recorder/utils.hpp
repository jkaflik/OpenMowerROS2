#pragma once

#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>

namespace open_mower_next::map_recorder::utils
{

// Check if a polygon is valid (at least 3 points and proper formation)
bool isValidPolygon(const std::vector<geometry_msgs::msg::Point>& points)
{
  if (points.size() < 3)
  {
    return false;
  }

  // Additional polygon validation could be added here
  // For example: check if the polygon is simple (non-self-intersecting)

  return true;
}

// Generate a unique ID
inline std::string generateUniqueId()
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(0, 15);
  static const char* hex_digits = "0123456789abcdef";

  std::string uuid;
  for (int i = 0; i < 32; ++i)
  {
    uuid += hex_digits[dis(gen)];
    if (i == 7 || i == 11 || i == 15 || i == 19)
    {
      uuid += '-';
    }
  }

  return uuid;
}

// Calculate Euclidean distance between two points
inline double pointDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Calculate Euclidean distance between two poses (ignoring orientation)
inline double poseDistance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
  return pointDistance(pose1.pose.position, pose2.pose.position);
}

}  // namespace open_mower_next::map_recorder::utils
