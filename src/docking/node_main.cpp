#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "docking/docking_action_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<open_mower_next::docking::DockingActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}