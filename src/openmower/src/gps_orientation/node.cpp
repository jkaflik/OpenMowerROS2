#include "node.hpp"

#include <robot_localization/navsat_conversions.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace OpenMower {
    namespace GPSOrientation {
        double normalize_angle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        Node::Node(const rclcpp::NodeOptions &options) : rclcpp::Node("gps_orientation", options) {
            min_gps_covariance_ = this->declare_parameter("min_gps_covariance", 0.03);
            min_gps_distance_ = this->declare_parameter("min_gps_distance", 0.1);

            gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "odometry/gps", 10, std::bind(&Node::gpsOdomCallback, this, std::placeholders::_1));

            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "diff_drive_base_controller/odom", 10, std::bind(&Node::odomCallback, this, std::placeholders::_1));

            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/gps/orientation", 10);
        }

        void Node::gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr gpsOdom) {
            if (gpsOdom->pose.covariance[0] > 0.0 && gpsOdom->pose.covariance[0] > min_gps_covariance_) {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                            "GPS odometry covariance is too high: " << gpsOdom->pose.covariance[0]
                                                                                    << " > "
                                                                                    << min_gps_covariance_);

                // drop the last known GPS odom, we don't want to use it if covariance became too high
                last_known_gps_odom_ = nullptr;

                return;
            }

            if (!last_known_gps_odom_) {
                last_known_gps_odom_ = gpsOdom;
                return;
            }

            // calculate distance between two points
            double distance = sqrt(pow(gpsOdom->pose.pose.position.x - last_known_gps_odom_->pose.pose.position.x, 2) +
                                   pow(gpsOdom->pose.pose.position.y - last_known_gps_odom_->pose.pose.position.y, 2));

            if (distance < min_gps_distance_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                     "Distance between two poses is too small. Is the robot moving?");
                return;
            }


            nav_msgs::msg::Odometry msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = gpsOdom->header.frame_id;
            msg.pose = gpsOdom->pose;
            msg.twist = gpsOdom->twist;

            msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));

            odom_publisher_->publish(msg);

            last_known_gps_odom_ = gpsOdom;
        }

        void Node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
            last_known_odom_ = odom;
        }
    } // OpenMower
} // DatumPublisher