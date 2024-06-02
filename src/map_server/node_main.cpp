#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "map_server/map_server_node.hpp"

using open_mower_next::map_server::MapServerNode;

int main(int argc, char **argv) {
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MapServerNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}