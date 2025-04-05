#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "open_mower_next/action/dock.hpp"
#include "open_mower_next/msg/map.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

namespace open_mower_next::docking {

class DockingActionServer : public rclcpp::Node {
public:
    using Dock = open_mower_next::action::Dock;
    using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
    using DockRobot = opennav_docking_msgs::action::DockRobot;

    explicit DockingActionServer(const rclcpp::NodeOptions& options);

private:
    // Action server
    rclcpp_action::Server<Dock>::SharedPtr action_server_;
    
    // Action client for Docking Server
    rclcpp_action::Client<DockRobot>::SharedPtr docking_client_;
    
    // TF for transforms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Map subscription to get docking stations
    rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_sub_;
    open_mower_next::msg::Map::SharedPtr current_map_;
    
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Dock::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDock> goal_handle);
    
    void handle_accepted(
        const std::shared_ptr<GoalHandleDock> goal_handle);
    
    // Execution method
    void execute(const std::shared_ptr<GoalHandleDock> goal_handle);
    
    // Helper methods
    open_mower_next::msg::DockingStation find_nearest_docking_station(
        const geometry_msgs::msg::PoseStamped& robot_pose);
    
    double calculate_distance(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2);
    
    // Map callback
    void map_callback(const open_mower_next::msg::Map::SharedPtr msg);
};

} // namespace open_mower_next::docking