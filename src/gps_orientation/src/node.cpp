#include "node.hpp"

#include <robot_localization/navsat_conversions.hpp>
#include "tf2/LinearMath/Quaternion.h"
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

            fix_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                    "gps/fix", 10, std::bind(&Node::fixCallback, this, std::placeholders::_1));
        }

        void Node::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix) {
            if (fix->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                     "GPS fix is not valid.");
                return;
            }

            if (fix->position_covariance[0] > 0.0 && fix->position_covariance[0] > min_gps_covariance_) {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                            "GPS odometry covariance is too high: " << fix->position_covariance[0]
                                                                                    << " > "
                                                                                    << min_gps_covariance_);

                // drop the last known GPS fix, we don't want to use it if covariance became too high
                last_known_fix_ = nullptr;

                return;
            }

            if (!last_known_fix_) {
                last_known_fix_ = fix;
                return;
            }

            std::string utm_zone_tmp;
            double x, y, last_x, last_y;
            robot_localization::navsat_conversions::LLtoUTM(fix->latitude, fix->longitude, x, y, utm_zone_tmp);
            robot_localization::navsat_conversions::LLtoUTM(last_known_fix_->latitude, last_known_fix_->longitude,
                                                            last_x, last_y, utm_zone_tmp);

            // calculate distance between two points
            double distance = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));

            if (distance < min_gps_distance_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                     "Distance between two poses is too small. Is the robot moving?");
                return;
            }

            // calculate theta that report 0 heading east
            double theta = normalize_angle(atan2(y - last_y, x - last_x) - M_PI_2);

            sensor_msgs::msg::Imu msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = fix->header.frame_id;
            msg.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
            msg.orientation_covariance[8] = (last_known_fix_->position_covariance[0] + fix->position_covariance[0]) / 2;
            msg.linear_acceleration_covariance[0] = -1;
            msg.angular_velocity_covariance[0] = -1;

            last_known_fix_ = fix;
        }
    } // OpenMower
} // GPSOrientation
