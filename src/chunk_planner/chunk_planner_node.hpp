#pragma once

#include <rclcpp/rclcpp.hpp>

namespace open_mower_next::chunk_planner {
    class ChunkPlannerNode final : public rclcpp::Node {
    public:
        explicit ChunkPlannerNode(const rclcpp::NodeOptions &options);

        ~ChunkPlannerNode() override = default;
    };
}
