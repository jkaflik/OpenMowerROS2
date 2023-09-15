#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "node.hpp"

int main(int argc, char **argv) {
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OpenMower::DatumPublisher::Node>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}