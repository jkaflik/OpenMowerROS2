#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "chunk_planner/chunk_planner_node.hpp"

using open_mower_next::chunk_planner::ChunkPlannerNode;

int main(int argc, char **argv) {
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ChunkPlannerNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}