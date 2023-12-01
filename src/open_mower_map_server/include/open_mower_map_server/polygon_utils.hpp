#pragma once

#include <geometry_msgs/msg/polygon.hpp>

inline double polygonArea(const geometry_msgs::msg::Polygon& polygon)
{
    double area = 0;

    for (size_t i = 0; i < polygon.points.size(); i++)
    {
        const auto& p1 = polygon.points[i];
        const auto& p2 = polygon.points[(i + 1) % polygon.points.size()];

        area += (p1.x * p2.y - p2.x * p1.y);
    }

    return area / 2;
}