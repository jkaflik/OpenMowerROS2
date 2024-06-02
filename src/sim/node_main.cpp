#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sim_node.hpp"

using open_mower_next::sim::SimNode;

int main(int argc, char **argv) {
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SimNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}