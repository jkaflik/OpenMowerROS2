#include "node.hpp"

#include <robot_localization/srv/set_datum.hpp>

namespace OpenMower {
    namespace DatumPublisher {
        Node::Node(const rclcpp::NodeOptions &options) : rclcpp::Node("datum_publisher", options) {
            datum_latitude_ = this->declare_parameter("datum.latitude", rclcpp::PARAMETER_DOUBLE);
            datum_longitude_ = this->declare_parameter("datum.longitude", rclcpp::PARAMETER_DOUBLE);

            try {
                RCLCPP_INFO(this->get_logger(), "Datum latitude: %f", datum_latitude_.get<double>());
                RCLCPP_INFO(this->get_logger(), "Datum longitude: %f", datum_longitude_.get<double>());
            } catch (const rclcpp::ParameterTypeException &e) {
                RCLCPP_ERROR(this->get_logger(), "Please provide both datum.latitude and datum.longitude parameters");
                rclcpp::shutdown();
                return;
            }

            setDatum();

            if (!this->declare_parameter("datum.publish_as_fix", false)) {
                rclcpp::shutdown();
                return;
            }

            std::string topic_name = this->declare_parameter("datum.fix_topic", "datum/fix");

            navsat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, 10);
            publishNavSatFix();
            // create timer and publish navsat fix every 10 seconds due to latching is not implemented well in ROS2
            auto timer = this->create_wall_timer(std::chrono::seconds(10), [this]() {
                publishNavSatFix();
            });
        }

        void Node::setDatum() {
            rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr setDatumClient =
                    this->create_client<robot_localization::srv::SetDatum>("/datum");

            while (!setDatumClient->wait_for_service(std::chrono::seconds(2))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_datum service. Exiting.");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "set_datum service not available, waiting again...");
            }

            auto request = robot_localization::srv::SetDatum::Request();
            request.geo_pose.position.latitude = datum_latitude_.get<double>();
            request.geo_pose.position.longitude = datum_longitude_.get<double>();

            auto result = setDatumClient->async_send_request(std::make_shared<robot_localization::srv::SetDatum::Request>(request));

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set datum");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Datum set successfully");
        }

        void Node::publishNavSatFix() {
            sensor_msgs::msg::NavSatFix msg;
            msg.latitude = datum_latitude_.get<double>();
            msg.longitude = datum_longitude_.get<double>();
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";

            navsat_fix_publisher_->publish(msg);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Published NavSatFix for datum");
        }
    } // OpenMower
} // DatumPublisher