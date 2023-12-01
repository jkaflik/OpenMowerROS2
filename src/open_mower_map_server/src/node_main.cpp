#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "open_mower_map_server/map_server_node.hpp"

int main(int argc, char **argv) {
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<open_mower_map_server::MapServerNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}