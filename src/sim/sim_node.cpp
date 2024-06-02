#include "sim_node.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

open_mower_next::sim::SimNode::SimNode(const rclcpp::NodeOptions &options): Node("sim_node", options) {
    model_tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/model/openmower/pose",
            10,
            std::bind(&SimNode::modelTfCallback, this, std::placeholders::_1)
    );

    map_subscriber_ = this->create_subscription<msg::Map>(
            "/map",
            10,
            std::bind(&SimNode::mapCallback, this, std::placeholders::_1)
    );

  charger_present_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/charger_present", 10);
  battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

  auto freq_ = this->declare_parameter<int32_t>("charger_simulation_freq", 15);
  this->create_timer(
      std::chrono::milliseconds(1000 / freq_),
      [this] { chargerPresentSimulationCallback(); }
      );

  battery_state_voltage_drop_per_second_ = this->declare_parameter<double>("battery_voltage_drop_per_second", 0.005); // defaults to 0.005V/s
  battery_state_voltage_charge_per_second_ = this->declare_parameter<double>("battery_voltage_charge_per_second", 0.01); // defaults to 0.01V/s

  battery_state_max_voltage_ = this->declare_parameter<double>("battery_max_voltage", 28.7); // defaults to 28.7V
  battery_state_min_voltage_ = this->declare_parameter<double>("battery_min_voltage", 21.7); // defaults to 21.7V

  battery_state_msg_.voltage = this->declare_parameter<double>("initial_battery_voltage", battery_state_max_voltage_); // defaults to max voltage
  battery_state_msg_.design_capacity = this->declare_parameter<double>("battery_design_capacity", 2.0); // defalts to 2Ah
  battery_state_msg_.capacity = this->declare_parameter<double>("initial_battery_capacity", battery_state_msg_.design_capacity); // defaults to design capacity

  battery_state_msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  battery_state_msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_msg_.present = true;

  this->create_timer(
  std::chrono::seconds(1),
  [this] { batteryStateSimulationCallback(); }
  );

    RCLCPP_INFO(get_logger(), "SimNode created");
}

// This function checks if the model pose is in a docking station.
// Transformations and comparisons are done in 2D. The model pose is considered to be in a docking station if:
// - The x and y distance to the docking station is less than 5cm
// - The yaw difference to the docking station is less than 0.25 rad
// It requires the map and the model pose to be up to date.
bool open_mower_next::sim::SimNode::isInDockingStation() {
  if (!map_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No map received yet");
    return false;
  }

  if (!model_baselink_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No model baselink pose received yet");
    return false;
  }

  // check if pose is up to date
  if (model_baselink_pose_->header.stamp.sec < get_clock()->now().seconds() - 1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Model baselink pose is outdated");
    return false;
  }

  bool inDockingStation = false;
  for (auto &dockingStation : map_->docking_stations) {
    // calculate distance and yaw between docking station and model pose

    tf2::Transform tf1, tf2;
    tf2::fromMsg(model_baselink_pose_->pose, tf1);
    tf2::fromMsg(dockingStation.pose.pose, tf2);

    auto relativeTransform = tf1.inverseTimes(tf2);
    auto translation = relativeTransform.getOrigin();
    auto rotation = relativeTransform.getRotation();

    double roll, pitch, yaw;
    tf2::Matrix3x3(rotation).getEulerYPR(yaw, pitch, roll);

    inDockingStation = std::abs(translation.x()) < 0.05 && // 5cm
                   std::abs(translation.y()) < 0.05 && // 5cm
                   std::abs(yaw) < 0.25; // 0.25 rad

    if (inDockingStation) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Contact with docking station %s", dockingStation.name.c_str());
      return true;
    }
  }
  return false;
}

void open_mower_next::sim::SimNode::chargerPresentSimulationCallback(){
  charger_present_msg_.data = isInDockingStation();
  charger_present_publisher_->publish(charger_present_msg_);
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
void open_mower_next::sim::SimNode::batteryStateSimulationCallback(){
  const auto now = get_clock()->now();

  if (last_battery_voltage_update_.seconds() == 0) {
    last_battery_voltage_update_ = now;
  }

  if (!charger_present_msg_.data) {
    auto sinceLastVoltageUpdate = (now - last_battery_voltage_update_).seconds();
    if (sinceLastVoltageUpdate >= 1) {
      battery_state_msg_.voltage -= sinceLastVoltageUpdate * battery_state_voltage_drop_per_second_;
    }
  } else {
    battery_state_msg_.voltage = std::min(battery_state_max_voltage_, battery_state_msg_.voltage + battery_state_voltage_charge_per_second_);
  }

  last_battery_voltage_update_ = now;
  battery_state_msg_.percentage = (battery_state_msg_.voltage - battery_state_min_voltage_) / (battery_state_max_voltage_ - battery_state_min_voltage_) * 100.0;

  if (charger_present_msg_.data) {
    if (battery_state_msg_.percentage < 100.0) {
      battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else {
      battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    }
  }

  battery_state_msg_.power_supply_health = battery_state_msg_.present
        ? (battery_state_msg_.voltage < battery_state_min_voltage_
           ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD
           : (battery_state_msg_.voltage > battery_state_max_voltage_
              ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE
              : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD))
        : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

  battery_state_msg_.header.stamp = now;
  battery_state_publisher_->publish(battery_state_msg_);
}

void open_mower_next::sim::SimNode::mapCallback(msg::Map::SharedPtr msg){
  map_ = msg;
}

void open_mower_next::sim::SimNode::modelTfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    geometry_msgs::msg::Transform *mapTransform;

    for (auto transform : msg->transforms) {
      if (transform.header.frame_id == "map") {
        mapTransform = &transform.transform;
        break;
      }
    }

    if (!mapTransform) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "No map transform found");
      return;
    }

  if (!model_baselink_pose_) {
    model_baselink_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  }

  tf2::Transform tfTransform;
  tf2::fromMsg(*mapTransform, tfTransform);
    tf2::toMsg(tfTransform, model_baselink_pose_->pose);
}


