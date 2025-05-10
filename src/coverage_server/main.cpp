#include <rclcpp/rclcpp.hpp>
#include "coverage_server_node.hpp"

using open_mower_next::coverage_server::CoverageServerNode;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<CoverageServerNode>(options);

  RCLCPP_INFO(node->get_logger(), "Starting Coverage Server node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
