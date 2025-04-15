#include "sim_node.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

open_mower_next::sim::SimNode::SimNode(const rclcpp::NodeOptions& options) : Node("sim_node", options)
{
  // Initialize tf2 buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // We still need to listen for the docking station pose since it's not part of the tf tree
  docking_station_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/model/docking_station/pose", 10, std::bind(&SimNode::dockingStationPoseCallback, this, std::placeholders::_1));

  charger_present_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/power/charger_present", 10);
  battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/power", 10);
  charge_voltage_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/power/charge_voltage", 10);

  auto freq_ = this->declare_parameter<int32_t>("charger_simulation_freq", 15);

  battery_state_voltage_drop_per_second_ =
      this->declare_parameter<double>("battery_voltage_drop_per_second", 0.005);  // defaults to 0.005V/s
  battery_state_voltage_charge_per_second_ =
      this->declare_parameter<double>("battery_voltage_charge_per_second", 0.01);  // defaults to 0.01V/s

  battery_state_max_voltage_ = this->declare_parameter<double>("battery_max_voltage", 28.7);  // defaults to 28.7V
  battery_state_min_voltage_ = this->declare_parameter<double>("battery_min_voltage", 21.7);  // defaults to 21.7V

  battery_state_msg_.voltage = this->declare_parameter<double>("initial_battery_voltage",
                                                               battery_state_max_voltage_);  // defaults to max voltage
  battery_state_msg_.design_capacity =
      this->declare_parameter<double>("battery_design_capacity", 2.0);  // defalts to 2Ah
  battery_state_msg_.capacity = this->declare_parameter<double>(
      "initial_battery_capacity", battery_state_msg_.design_capacity);  // defaults to design capacity

  battery_state_msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  battery_state_msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_msg_.present = true;

  charger_timer_ =
      this->create_timer(std::chrono::milliseconds(1000 / freq_), [this] { chargerPresentSimulationCallback(); });

  battery_timer_ = this->create_timer(std::chrono::seconds(1), [this] { batteryStateSimulationCallback(); });

  // Docking station pose offset parameters
  // Defaults to an empty.sdf docking_station model
  docking_station_offset_x_ = this->declare_parameter<double>("docking_station_offset_x", 0.32);
  docking_station_offset_y_ = this->declare_parameter<double>("docking_station_offset_y", 0.0);

  RCLCPP_INFO(get_logger(), "Docking station pose offset: x=%f, y=%f", docking_station_offset_x_,
              docking_station_offset_y_);

  RCLCPP_INFO(get_logger(), "SimNode created with timer frequency: %d Hz", freq_);
}

bool open_mower_next::sim::SimNode::isInDockingStation()
{
  if (!docking_station_pose_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No docking station pose received yet");
    return false;
  }

  // Try to get the current charging port position
  geometry_msgs::msg::TransformStamped charging_port_transform;
  try
  {
    charging_port_transform = tf_buffer_->lookupTransform("map", "charging_port", tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not get charging port transform: %s", ex.what());
    return false;
  }

  // Convert transform to pose for comparison with docking station
  geometry_msgs::msg::Pose charging_port_pose;
  tf2::Transform tf_charging_port;
  tf2::fromMsg(charging_port_transform.transform, tf_charging_port);
  tf2::toMsg(tf_charging_port, charging_port_pose);

  // Calculate relative position and orientation between charging port and docking station
  tf2::Transform port_transform, dock_transform;
  tf2::fromMsg(charging_port_pose, port_transform);
  tf2::fromMsg(docking_station_pose_->pose, dock_transform);

  // Get relative transform from charging port to dock
  auto relativeTransform = port_transform.inverseTimes(dock_transform);
  auto translation = relativeTransform.getOrigin();

  // Check if charging port is close enough to the dock
  // Using stricter thresholds since we're checking actual charging port
  bool inDockingStation = std::abs(translation.x()) < 0.05 && std::abs(translation.y()) < 0.05;

  // Log the distance to the docking station
  double distance = std::sqrt(std::pow(translation.x(), 2) + std::pow(translation.y(), 2));

  // Calculate the angle between the charging port and the docking station

  double angle = std::atan2(translation.y(), translation.x());

  if (!inDockingStation && distance < 1.0)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Distance from charging port to docking station: %f m, angle: %f rad", distance, angle);
  }

  if (inDockingStation)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Charging port in contact with docking station");
    return true;
  }

  return false;
}

void open_mower_next::sim::SimNode::chargerPresentSimulationCallback()
{
  auto inDock = isInDockingStation();

  charger_present_msg_.data = inDock;
  charger_present_publisher_->publish(charger_present_msg_);

  if (inDock)
  {
    charge_voltage_msg_.data = battery_state_max_voltage_;
  }
  else
  {
    charge_voltage_msg_.data = 0.0;
  }

  charge_voltage_publisher_->publish(charge_voltage_msg_);
}

// This callback is called in a given interval to simulate the battery state
// Logic is really simple and doesn't reflect real battery and charging behavior:
// - If the charger is present, the battery is charged with a constant rate V/s
// - If the charger is not present, the battery is discharged with a constant rate V/s
// - The battery is considered full at 100% and empty at 0%
// - The battery is considered dead if the voltage drops below a certain threshold
// - The battery is considered overvoltage if the voltage exceeds a certain threshold
// - The battery health is considered good if the voltage is within the thresholds
// - The battery health is considered unknown if the battery is not present
// - The battery technology is considered Li-Ion
void open_mower_next::sim::SimNode::batteryStateSimulationCallback()
{
  const auto now = get_clock()->now();

  if (last_battery_voltage_update_.seconds() == 0)
  {
    last_battery_voltage_update_ = now;
  }

  if (!charger_present_msg_.data)
  {
    auto sinceLastVoltageUpdate = (now - last_battery_voltage_update_).seconds();
    if (sinceLastVoltageUpdate >= 1)
    {
      battery_state_msg_.voltage -= sinceLastVoltageUpdate * battery_state_voltage_drop_per_second_;
    }
  }
  else
  {
    battery_state_msg_.voltage =
        std::min(battery_state_max_voltage_, battery_state_msg_.voltage + battery_state_voltage_charge_per_second_);
  }

  last_battery_voltage_update_ = now;
  battery_state_msg_.percentage = (battery_state_msg_.voltage - battery_state_min_voltage_) /
                                  (battery_state_max_voltage_ - battery_state_min_voltage_) * 100.0;

  if (charger_present_msg_.data)
  {
    if (battery_state_msg_.percentage < 100.0)
    {
      battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    }
    else
    {
      battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    }
  }

  battery_state_msg_.power_supply_health =
      battery_state_msg_.present ? (battery_state_msg_.voltage < battery_state_min_voltage_ ?
                                        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD :
                                        (battery_state_msg_.voltage > battery_state_max_voltage_ ?
                                             sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE :
                                             sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD)) :
                                   sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

  battery_state_msg_.header.stamp = now;
  battery_state_publisher_->publish(battery_state_msg_);
}

void open_mower_next::sim::SimNode::dockingStationPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(get_logger(), "Received docking station pose message");

  if (!docking_station_pose_)
  {
    docking_station_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  }

  docking_station_pose_->header.stamp = this->get_clock()->now();

  // Apply the configured offsets to better represent the charging connector position
  docking_station_pose_->pose = *msg;

  // Apply offsets in the docking station's local coordinate frame
  // First, create a transform from the original pose
  tf2::Transform dockTransform;
  tf2::fromMsg(*msg, dockTransform);

  // Create an offset transform in the dock's local frame
  tf2::Transform offsetTransform;
  offsetTransform.setOrigin(tf2::Vector3(docking_station_offset_x_, docking_station_offset_y_, 0.0));
  offsetTransform.setRotation(tf2::Quaternion(0, 0, 0, 1));  // Identity rotation

  // Apply the offset in the dock's coordinate frame
  tf2::Transform adjustedTransform = dockTransform * offsetTransform;

  // Set the updated pose
  tf2::toMsg(adjustedTransform, docking_station_pose_->pose);

  RCLCPP_INFO_ONCE(get_logger(), "Original docking station pose: [%f, %f, %f]", msg->position.x, msg->position.y,
                   msg->position.z);
  RCLCPP_INFO_ONCE(get_logger(), "Adjusted docking station pose: [%f, %f, %f]", docking_station_pose_->pose.position.x,
                   docking_station_pose_->pose.position.y, docking_station_pose_->pose.position.z);
}
