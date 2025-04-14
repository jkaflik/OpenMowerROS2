#pragma once

#include <geometry_msgs/msg/polygon.hpp>
#include <cmath>

inline double polygonArea(const geometry_msgs::msg::Polygon& polygon)
{
  double area = 0;

  for (size_t i = 0; i < polygon.points.size(); i++)
  {
    const auto& p1 = polygon.points[i];
    const auto& p2 = polygon.points[(i + 1) % polygon.points.size()];

    area += (p1.x * p2.y - p2.x * p1.y);
  }

  return std::abs(area / 2);
}

inline std::vector<geometry_msgs::msg::Polygon> splitPolygonIntoParts(const geometry_msgs::msg::Polygon& polygon,
                                                                      double maxArea)
{
  std::vector<geometry_msgs::msg::Polygon> result;

  double area = polygonArea(polygon);
  if (area <= maxArea)
  {
    result.push_back(polygon);
  }
  else
  {
    // Find the longest side
    double maxDistance = 0;
    size_t index = 0;
    for (size_t i = 0; i < polygon.points.size(); i++)
    {
      const auto& p1 = polygon.points[i];
      const auto& p2 = polygon.points[(i + 1) % polygon.points.size()];

      double distance = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
      if (distance > maxDistance)
      {
        maxDistance = distance;
        index = i;
      }
    }

    // Split the polygon into two along the longest side
    geometry_msgs::msg::Polygon polygon1, polygon2;
    for (size_t i = 0; i <= index; i++)
    {
      polygon1.points.push_back(polygon.points[i]);
    }
    for (size_t i = index; i < polygon.points.size(); i++)
    {
      polygon2.points.push_back(polygon.points[i]);
    }
    polygon2.points.push_back(polygon.points[0]);

    // Recursively split the two polygons
    auto polygons1 = splitPolygonIntoParts(polygon1, maxArea);
    auto polygons2 = splitPolygonIntoParts(polygon2, maxArea);

    // Add the split polygons to the result
    result.insert(result.end(), polygons1.begin(), polygons1.end());
    result.insert(result.end(), polygons2.begin(), polygons2.end());
  }

  return result;
}
