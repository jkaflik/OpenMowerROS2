#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace OpenMower {
    namespace GPSOrientation {

        class Node final : public rclcpp::Node {
        public:
            explicit Node(const rclcpp::NodeOptions &options);

        private:
            double min_gps_covariance_;
            double min_gps_distance_;

            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscriber_;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr odom_publisher_;

            sensor_msgs::msg::NavSatFix::SharedPtr last_known_fix_;

            void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix);
        };
    } // OpenMower
} // DatumPublisher

