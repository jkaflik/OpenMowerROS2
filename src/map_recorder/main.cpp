#include "map_recorder/map_recorder_node.hpp"
#include <rclcpp/rclcpp.hpp>

using open_mower_next::map_recorder::MapRecorderNode;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<MapRecorderNode>(options);

  RCLCPP_INFO(node->get_logger(), "Starting Map Recorder node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
