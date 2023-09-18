#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace OpenMower {
    namespace DatumPublisher {

        class Node final : public rclcpp::Node {
        public:
            explicit Node(const rclcpp::NodeOptions &options);
        private:
            rclcpp::ParameterValue datum_latitude_;
            rclcpp::ParameterValue datum_longitude_;

            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_publisher_;

            void setDatum();
            void publishNavSatFix();
        };

    } // OpenMower
} // DatumPublisher

