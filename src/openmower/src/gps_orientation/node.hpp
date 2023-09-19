#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace OpenMower {
    namespace GPSOrientation {

        class Node final : public rclcpp::Node {
        public:
            explicit Node(const rclcpp::NodeOptions &options);

        private:
            double min_gps_covariance_;
            double min_gps_distance_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_subscriber_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

            nav_msgs::msg::Odometry::SharedPtr last_known_gps_odom_;
            nav_msgs::msg::Odometry::SharedPtr last_known_odom_;

            void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr gpsOdom);

            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
        };
    } // OpenMower
} // DatumPublisher

