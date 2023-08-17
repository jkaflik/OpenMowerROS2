#include "openmower_hw_comms/openmower_hw_comms.hpp"

openmower_hw_comms::OpenMowerHWComms::OpenMowerHWComms(const rclcpp::NodeOptions &options) : rclcpp::Node("openmower_hw_comms", options)
{
    // get OpenMower serial port address
    std::string port = declare_parameter<std::string>("port", "");
}