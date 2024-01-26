#pragma once

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>

namespace open_mower_next::map_server {

    class PolygonGridIterator {
    public:
        PolygonGridIterator(const geometry_msgs::msg::Polygon& polygon, double resolution)
                : polygon_(polygon), resolution_(resolution) {
            minimumBoundingRectangle();

            // Initialize x_, y_ to start from the bottom-left corner of the MBR
            x_ = min_x_;
            y_ = min_y_;
        }

        void minimumBoundingRectangle() {
            float min_x = std::numeric_limits<float>::infinity();
            float min_y = std::numeric_limits<float>::infinity();
            float max_x = -std::numeric_limits<float>::infinity();
            float max_y = -std::numeric_limits<float>::infinity();

            for (const auto& point : polygon_.points) {
                if (point.x < min_x) min_x = point.x;
                if (point.x > max_x) max_x = point.x;
                if (point.y < min_y) min_y = point.y;
                if (point.y > max_y) max_y = point.y;
            }

            min_x_ = min_x;
            min_y_ = min_y;
            max_x_ = max_x;
            max_y_ = max_y;
        }

        PolygonGridIterator(const PolygonGridIterator& other)
                : polygon_(other.polygon_), resolution_(other.resolution_), x_(other.x_), y_(other.y_) {}

        bool next() {
            while (x_ <= max_x_ && y_ <= max_y_) {
                x_ += resolution_;
                if (x_ > max_x_) {
                    x_ = min_x_;
                    y_++;
                }

                if (isPointInsidePolygon(x_, y_, polygon_.points)) {
                    return true;
                }
            }

            return false;
        }

        bool operator==(const PolygonGridIterator& other) const {
            return polygon_ == other.polygon_ && resolution_ == other.resolution_ && x_ == other.x_ && y_ == other.y_;
        }

        bool operator!=(const PolygonGridIterator& other) const {
            return !(*this == other);
        }

        const geometry_msgs::msg::Point operator*() const {
            auto p = geometry_msgs::msg::Point();
            p.x = x_;
            p.y = y_;
            p.z = 0;
            return p;
        }

    private:
        bool isPointInsidePolygon(float x, float y, const std::vector<geometry_msgs::msg::Point32>& vertices) {
            bool isInside = false;
            int n = vertices.size();

            // Loop through all edges of the polygon
            for (int i = 0, j = n - 1; i < n; j = i++) {
                float xi = vertices[i].x, yi = vertices[i].y;
                float xj = vertices[j].x, yj = vertices[j].y;

                // Check if the point is in one of the edges
                if (((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                    isInside = !isInside;
                }
            }

            return isInside;
        }

        const geometry_msgs::msg::Polygon& polygon_;
        double resolution_;

        double x_;
        double y_;
        float min_x_;
        float min_y_;
        float max_x_;
        float max_y_;
    };

}
