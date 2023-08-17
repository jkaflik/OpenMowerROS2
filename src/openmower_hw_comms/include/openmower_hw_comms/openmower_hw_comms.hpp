#pragma once

#include <rclcpp/rclcpp.hpp>

namespace openmower_hw_comms
{
    class OpenMowerHWComms : public rclcpp::Node
    {
        public:
            explicit OpenMowerHWComms(const rclcpp::NodeOptions & options);
    };
}
