#pragma once

#include <geometry_msgs/msg/polygon.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

inline double polygonArea(const geometry_msgs::msg::Polygon& polygon)
{
  double area = 0.0;

  if (polygon.points.size() < 3)
  {
    return 0.0;
  }

  for (size_t i = 0; i < polygon.points.size(); i++)
  {
    const auto& p1 = polygon.points[i];
    const auto& p2 = polygon.points[(i + 1) % polygon.points.size()];

    area += (p1.x * p2.y - p2.x * p1.y);
  }

  return std::abs(area / 2.0);
}

// Find the longest edge in the polygon
inline std::pair<size_t, double> findLongestEdge(const geometry_msgs::msg::Polygon& polygon)
{
  double maxDistance = 0.0;
  size_t maxIndex = 0;

  for (size_t i = 0; i < polygon.points.size(); i++)
  {
    const auto& p1 = polygon.points[i];
    const auto& p2 = polygon.points[(i + 1) % polygon.points.size()];

    double distance = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    if (distance > maxDistance)
    {
      maxDistance = distance;
      maxIndex = i;
    }
  }

  return { maxIndex, maxDistance };
}

inline std::vector<geometry_msgs::msg::Polygon> splitPolygonIntoParts(const geometry_msgs::msg::Polygon& polygon,
                                                                      double maxArea)
{
  std::vector<geometry_msgs::msg::Polygon> result;

  // Handle empty or invalid polygons
  if (polygon.points.size() < 3)
  {
    if (!polygon.points.empty())
    {
      // Return the original invalid polygon
      result.push_back(polygon);
    }
    return result;
  }

  double area = polygonArea(polygon);

  // If area is small enough, return the original polygon
  if (area <= maxArea)
  {
    result.push_back(polygon);
    return result;
  }

  // Find the longest edge
  auto [edgeIndex, maxDistance] = findLongestEdge(polygon);

  // Create the first half of the split
  geometry_msgs::msg::Polygon polygon1;

  // Get the midpoint of the longest edge
  size_t nextIndex = (edgeIndex + 1) % polygon.points.size();
  geometry_msgs::msg::Point32 midpoint;
  midpoint.x = (polygon.points[edgeIndex].x + polygon.points[nextIndex].x) / 2;
  midpoint.y = (polygon.points[edgeIndex].y + polygon.points[nextIndex].y) / 2;

  // Find the point farthest from the longest edge to make a better split
  double maxDist = 0;
  size_t farPointIndex = 0;
  for (size_t i = 0; i < polygon.points.size(); i++)
  {
    if (i != edgeIndex && i != nextIndex)
    {
      double dist = std::pow(polygon.points[i].x - midpoint.x, 2) + std::pow(polygon.points[i].y - midpoint.y, 2);
      if (dist > maxDist)
      {
        maxDist = dist;
        farPointIndex = i;
      }
    }
  }

  // Now split the polygon along the line from midpoint to farPoint
  geometry_msgs::msg::Polygon leftPoly, rightPoly;

  // Start with the midpoint
  leftPoly.points.push_back(midpoint);

  // Add the farthest point
  leftPoly.points.push_back(polygon.points[farPointIndex]);

  // Add points going around the polygon in one direction until we reach the edge again
  for (size_t i = (farPointIndex + 1) % polygon.points.size(); i != (nextIndex % polygon.points.size());
       i = (i + 1) % polygon.points.size())
  {
    leftPoly.points.push_back(polygon.points[i]);
  }

  // Close with the midpoint
  leftPoly.points.push_back(midpoint);

  // Now build the right polygon
  rightPoly.points.push_back(midpoint);

  // Add points going the other way around
  for (size_t i = nextIndex; i != farPointIndex; i = (i + 1) % polygon.points.size())
  {
    rightPoly.points.push_back(polygon.points[i]);
  }

  // Close with the midpoint
  rightPoly.points.push_back(midpoint);

  // Recursively split both halves
  auto leftSplits = splitPolygonIntoParts(leftPoly, maxArea);
  auto rightSplits = splitPolygonIntoParts(rightPoly, maxArea);

  // Combine the results
  result.insert(result.end(), leftSplits.begin(), leftSplits.end());
  result.insert(result.end(), rightSplits.begin(), rightSplits.end());

  return result;
}
