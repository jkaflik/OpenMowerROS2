#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "open_mower_next/msg/map.hpp"
#include "map_server/some_gaussian_filter.hpp"
#include "open_mower_next/srv/save_area.hpp"
#include "open_mower_next/srv/remove_area.hpp"
#include "open_mower_next/srv/save_docking_station.hpp"
#include "open_mower_next/srv/remove_docking_station.hpp"

namespace open_mower_next::map_server
{
class MapIO
{
public:
  virtual ~MapIO() = default;

  virtual void save(msg::Map map) = 0;

  virtual msg::Map load() = 0;
};

class MapServerNode final : public rclcpp::Node
{
public:
  void publishMap();
  explicit MapServerNode(const rclcpp::NodeOptions& options);

  ~MapServerNode() override = default;

private:
  std::vector<msg::Area> areasWithExclusionsLast(std::vector<msg::Area> areas);
  nav_msgs::msg::OccupancyGrid mapToOccupancyGrid(msg::Map map);
  visualization_msgs::msg::MarkerArray mapToVisualizationMarkers(msg::Map map);
  geometry_msgs::msg::PoseArray dockingStationsToPoseArray(msg::Map map);

  MapIO* map_io_;

  void fillGridWithPolygon(nav_msgs::msg::OccupancyGrid& occupancy_grid, const geometry_msgs::msg::Polygon& polygon,
                           uint8_t value);

  // parameters
  std::string map_type_;
  std::string map_file_;
  bool use_gaussian_blur_;

  void configureGaussianBlur();

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  void configureMap();
  void saveAndPublishMap();

  SomeGaussianFilter* gaussian_filter_;
  msg::Map current_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<msg::Map>::SharedPtr map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_visualization_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr docking_station_poses_publisher_;

  rclcpp::Service<srv::SaveArea>::SharedPtr save_area_service_;
  rclcpp::Service<srv::RemoveArea>::SharedPtr remove_area_service_;
  rclcpp::Service<srv::SaveDockingStation>::SharedPtr save_docking_station_service_;
  rclcpp::Service<srv::RemoveDockingStation>::SharedPtr remove_docking_station_service_;

  void configureServices();

  void saveAreaHandler(srv::SaveArea::Request::SharedPtr request, srv::SaveArea::Response::SharedPtr response);
  void removeAreaHandler(srv::RemoveArea::Request::SharedPtr request, srv::RemoveArea::Response::SharedPtr response);
  void saveDockingStationHandler(srv::SaveDockingStation::Request::SharedPtr request,
                                 srv::SaveDockingStation::Response::SharedPtr response);
  void removeDockingStationHandler(srv::RemoveDockingStation::Request::SharedPtr request,
                                   srv::RemoveDockingStation::Response::SharedPtr response);
};
}  // namespace open_mower_next::map_server
