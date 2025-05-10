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

inline bool polygonsIntersect(const geometry_msgs::msg::Polygon& poly1, const geometry_msgs::msg::Polygon& poly2)
{
  for (size_t i = 0; i < poly1.points.size(); i++)
  {
    const auto& p1 = poly1.points[i];
    const auto& p2 = poly1.points[(i + 1) % poly1.points.size()];

    for (size_t j = 0; j < poly2.points.size(); j++)
    {
      const auto& q1 = poly2.points[j];
      const auto& q2 = poly2.points[(j + 1) % poly2.points.size()];

      double det = (p2.x - p1.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q2.x - q1.x);
      if (det == 0)
      {
        continue;
      }

      double t = ((p1.x - q1.x) * (q2.y - q1.y) - (p1.y - q1.y) * (q2.x - q1.x)) / det;
      double u = ((p1.x - q1.x) * (p2.y - p1.y) - (p1.y - p1.y) * (p2.x - p1.x)) / det;

      if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
      {
        return true;
      }
    }
  }

  return false;
}

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

  if (polygon.points.size() < 3)
  {
    if (!polygon.points.empty())
    {
      result.push_back(polygon);
    }
    return result;
  }

  double area = polygonArea(polygon);

  if (area <= maxArea)
  {
    result.push_back(polygon);
    return result;
  }

  auto [edgeIndex, maxDistance] = findLongestEdge(polygon);

  geometry_msgs::msg::Polygon polygon1;

  size_t nextIndex = (edgeIndex + 1) % polygon.points.size();
  geometry_msgs::msg::Point32 midpoint;
  midpoint.x = (polygon.points[edgeIndex].x + polygon.points[nextIndex].x) / 2;
  midpoint.y = (polygon.points[edgeIndex].y + polygon.points[nextIndex].y) / 2;

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

  geometry_msgs::msg::Polygon leftPoly, rightPoly;

  leftPoly.points.push_back(midpoint);

  leftPoly.points.push_back(polygon.points[farPointIndex]);

  for (size_t i = (farPointIndex + 1) % polygon.points.size(); i != (nextIndex % polygon.points.size());
       i = (i + 1) % polygon.points.size())
  {
    leftPoly.points.push_back(polygon.points[i]);
  }

  leftPoly.points.push_back(midpoint);

  rightPoly.points.push_back(midpoint);

  for (size_t i = nextIndex; i != farPointIndex; i = (i + 1) % polygon.points.size())
  {
    rightPoly.points.push_back(polygon.points[i]);
  }

  rightPoly.points.push_back(midpoint);

  auto leftSplits = splitPolygonIntoParts(leftPoly, maxArea);
  auto rightSplits = splitPolygonIntoParts(rightPoly, maxArea);

  result.insert(result.end(), leftSplits.begin(), leftSplits.end());
  result.insert(result.end(), rightSplits.begin(), rightSplits.end());

  return result;
}
