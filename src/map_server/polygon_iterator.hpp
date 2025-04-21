#pragma once

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

namespace open_mower_next::map_server
{

class PolygonGridIterator
{
public:
  PolygonGridIterator(const geometry_msgs::msg::Polygon& polygon, double resolution)
    : polygon_(polygon), resolution_(resolution)
  {
    calculateBoundingBox();

    // Initialize iterator to start from the bottom-left corner of the MBR
    x_ = min_x_;
    y_ = min_y_;

    precomputeEdgeData();
  }

  void calculateBoundingBox()
  {
    min_x_ = std::numeric_limits<float>::max();
    min_y_ = std::numeric_limits<float>::max();
    max_x_ = std::numeric_limits<float>::lowest();
    max_y_ = std::numeric_limits<float>::lowest();

    for (const auto& point : polygon_.points)
    {
      min_x_ = std::min(min_x_, point.x);
      max_x_ = std::max(max_x_, point.x);
      min_y_ = std::min(min_y_, point.y);
      max_y_ = std::max(max_y_, point.y);
    }
  }

  void precomputeEdgeData()
  {
    const int n = polygon_.points.size();
    edge_data_.resize(n);

    for (int i = 0; i < n; i++)
    {
      int j = (i + 1) % n;
      const auto& p1 = polygon_.points[i];
      const auto& p2 = polygon_.points[j];

      edge_data_[i].x1 = p1.x;
      edge_data_[i].y1 = p1.y;
      edge_data_[i].x2 = p2.x;
      edge_data_[i].y2 = p2.y;
      edge_data_[i].dx = p2.x - p1.x;
      edge_data_[i].dy = p2.y - p1.y;
    }
  }

  PolygonGridIterator(const PolygonGridIterator& other)
    : polygon_(other.polygon_)
    , resolution_(other.resolution_)
    , edge_data_(other.edge_data_)
    , x_(other.x_)
    , y_(other.y_)
    , min_x_(other.min_x_)
    , min_y_(other.min_y_)
    , max_x_(other.max_x_)
    , max_y_(other.max_y_)
  {
  }

  bool next()
  {
    while (true)
    {
      x_ += resolution_;
      if (x_ > max_x_)
      {
        x_ = min_x_;
        y_ += resolution_;
        if (y_ > max_y_)
        {
          return false;
        }
      }

      if (isPointInsidePolygon(x_, y_))
      {
        return true;
      }
    }
  }

  bool operator==(const PolygonGridIterator& other) const
  {
    return polygon_ == other.polygon_ && resolution_ == other.resolution_ && x_ == other.x_ && y_ == other.y_;
  }

  bool operator!=(const PolygonGridIterator& other) const
  {
    return !(*this == other);
  }

  const geometry_msgs::msg::Point operator*() const
  {
    auto p = geometry_msgs::msg::Point();
    p.x = x_;
    p.y = y_;
    p.z = 0;
    return p;
  }

private:
  // Structure to store precomputed edge data
  struct EdgeData
  {
    float x1, y1, x2, y2;
    float dx, dy;
  };

  bool isPointInsidePolygon(float x, float y)
  {
    // Raycasting algorithm - count number of intersections
    bool inside = false;

    for (const auto& edge : edge_data_)
    {
      // Check if point is on an edge (simplified version)
      float dist_to_line = std::abs((edge.dy * x - edge.dx * y + edge.x2 * edge.y1 - edge.y2 * edge.x1) /
                                    std::sqrt(edge.dx * edge.dx + edge.dy * edge.dy));

      if (dist_to_line < resolution_ * 0.1f && x >= std::min(edge.x1, edge.x2) - resolution_ * 0.1f &&
          x <= std::max(edge.x1, edge.x2) + resolution_ * 0.1f &&
          y >= std::min(edge.y1, edge.y2) - resolution_ * 0.1f && y <= std::max(edge.y1, edge.y2) + resolution_ * 0.1f)
      {
        return true;  // Point is on the edge
      }

      // Standard ray casting algorithm
      if (((edge.y1 > y) != (edge.y2 > y)) && (x < (edge.dx) * (y - edge.y1) / (edge.dy) + edge.x1))
      {
        inside = !inside;
      }
    }

    return inside;
  }

  const geometry_msgs::msg::Polygon& polygon_;
  double resolution_;
  std::vector<EdgeData> edge_data_;

  double x_;
  double y_;
  float min_x_;
  float min_y_;
  float max_x_;
  float max_y_;
};

}  // namespace open_mower_next::map_server
