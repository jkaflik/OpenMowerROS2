#include "docking/docking_action_server.hpp"

namespace open_mower_next::docking {

DockingActionServer::DockingActionServer(const rclcpp::NodeOptions& options)
: Node("docking_action_server", options) {
    
    RCLCPP_INFO(get_logger(), "Starting Docking Action Server");
    
    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribe to the map topic to get docking stations
    map_sub_ = this->create_subscription<open_mower_next::msg::Map>(
        "/map", 10, 
        std::bind(&DockingActionServer::map_callback, this, std::placeholders::_1));
    
    // Initialize action client for docking server with a longer timeout
    docking_client_ = rclcpp_action::create_client<DockRobot>(
        this, "docking_server/dock");

    // Initialize action server
    action_server_ = rclcpp_action::create_server<Dock>(
        this,
        "dock",
        std::bind(&DockingActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DockingActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&DockingActionServer::handle_accepted, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Docking Action Server initialized");
}

void DockingActionServer::map_callback(const open_mower_next::msg::Map::SharedPtr msg) {
    current_map_ = msg;
    RCLCPP_DEBUG(get_logger(), "Received map with %zu docking stations", 
        current_map_->docking_stations.size());
}

rclcpp_action::GoalResponse DockingActionServer::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Dock::Goal>) {
    
    // Check if the map is available
    if (!current_map_) {
        RCLCPP_ERROR(get_logger(), "No map available, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if there are any docking stations
    if (current_map_->docking_stations.empty()) {
        RCLCPP_ERROR(get_logger(), "No docking stations in map, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if docking client is available (wait longer to make sure it connects)
//    if (!docking_client_->wait_for_action_server(std::chrono::seconds(5))) {
//        RCLCPP_ERROR(get_logger(), "Docking server not available, rejecting goal");
//        return rclcpp_action::GoalResponse::REJECT;
//    }
//    RCLCPP_INFO(get_logger(), "Connected to docking server");
    
    RCLCPP_INFO(get_logger(), "Accepted docking goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleDock>) {
    
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleDock> goal_handle) {
    
    // Start a new thread to execute the action
    std::thread{std::bind(&DockingActionServer::execute, this, goal_handle)}.detach();
}

open_mower_next::msg::DockingStation DockingActionServer::find_nearest_docking_station(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
    
    open_mower_next::msg::DockingStation nearest_dock;
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& dock : current_map_->docking_stations) {
        try {
            // Transform dock pose to robot frame if needed
            geometry_msgs::msg::PoseStamped dock_pose_transformed;
            
            if (dock.pose.header.frame_id != robot_pose.header.frame_id) {
                tf_buffer_->transform(
                    dock.pose, 
                    dock_pose_transformed, 
                    robot_pose.header.frame_id);
            } else {
                dock_pose_transformed = dock.pose;
            }
            
            // Calculate distance
            double distance = calculate_distance(
                robot_pose.pose.position, 
                dock_pose_transformed.pose.position);
            
            if (distance < min_distance) {
                min_distance = distance;
                nearest_dock = dock;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "Could not transform dock pose: %s", ex.what());
        }
    }
    
    RCLCPP_INFO(get_logger(), "Found nearest docking station: %s (%s) at distance %.2f meters", 
        nearest_dock.id.c_str(), nearest_dock.name.c_str(), min_distance);
    
    return nearest_dock;
}

double DockingActionServer::calculate_distance(
    const geometry_msgs::msg::Point& p1, 
    const geometry_msgs::msg::Point& p2) {
    
    return std::sqrt(
        std::pow(p1.x - p2.x, 2) + 
        std::pow(p1.y - p2.y, 2) + 
        std::pow(p1.z - p2.z, 2));
}

void DockingActionServer::execute(
    const std::shared_ptr<GoalHandleDock> goal_handle) {
    
    RCLCPP_INFO(get_logger(), "Executing dock action");
    
    // Initialize feedback and result
    auto feedback = std::make_shared<Dock::Feedback>();
    auto result = std::make_shared<Dock::Result>();
    result->success = false;
    result->error_code = Dock::Result::NONE;
    
    // Get current robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = this->now();
    
    try {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Could not get robot pose: %s", ex.what());
        result->error_code = Dock::Result::UNKNOWN;
        goal_handle->abort(result);
        return;
    }
    
    // Find nearest docking station
    if (!current_map_ || current_map_->docking_stations.empty()) {
        RCLCPP_ERROR(get_logger(), "No docking stations available");
        result->error_code = Dock::Result::NO_DOCK_AVAILABLE;
        goal_handle->abort(result);
        return;
    }
    
    open_mower_next::msg::DockingStation nearest_dock = find_nearest_docking_station(robot_pose);
    
    if (nearest_dock.id.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to find nearest docking station");
        result->error_code = Dock::Result::NO_DOCK_AVAILABLE;
        goal_handle->abort(result);
        return;
    }
    
    // Create docking goal
    auto dock_goal = DockRobot::Goal();
    dock_goal.use_dock_id = false;  // We'll use the dock pose directly
    dock_goal.dock_pose = nearest_dock.pose;
    dock_goal.dock_type = "openmower_dock";  // Match the plugin name in nav2_params.yaml
    dock_goal.max_staging_time = 300.0;  // 5 minutes
    dock_goal.navigate_to_staging_pose = true;
    
    RCLCPP_INFO(get_logger(), "Sending dock request to docking server for dock: %s (%s)",
                nearest_dock.id.c_str(), nearest_dock.name.c_str());
    
    // Send goal to docking server
    auto send_goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
    
    // Define callback for goal response
    send_goal_options.goal_response_callback = 
        [this](const rclcpp_action::ClientGoalHandle<DockRobot>::SharedPtr& goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Docking goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Docking goal accepted by server");
            }
        };
    
    // Define callback for feedback
    send_goal_options.feedback_callback = 
        [this, goal_handle](
            rclcpp_action::ClientGoalHandle<DockRobot>::SharedPtr,
            const std::shared_ptr<const DockRobot::Feedback> feedback) {
            
            // Create our action's feedback from docking server feedback
            auto fb = std::make_shared<Dock::Feedback>();
            fb->state = feedback->state;
            fb->docking_time = feedback->docking_time;
            fb->num_retries = feedback->num_retries;
            
            // Publish feedback
            goal_handle->publish_feedback(fb);
            
            // Log feedback
            std::string state_str;
            switch (feedback->state) {
                case DockRobot::Feedback::NAV_TO_STAGING_POSE:
                    state_str = "Navigating to staging pose";
                    break;
                case DockRobot::Feedback::INITIAL_PERCEPTION:
                    state_str = "Initial perception";
                    break;
                case DockRobot::Feedback::CONTROLLING:
                    state_str = "Controlling";
                    break;
                case DockRobot::Feedback::WAIT_FOR_CHARGE:
                    state_str = "Waiting for charge";
                    break;
                case DockRobot::Feedback::RETRY:
                    state_str = "Retrying";
                    break;
                default:
                    state_str = "Unknown";
            }
            
            RCLCPP_INFO(this->get_logger(), "Docking state: %s, retries: %d", 
                state_str.c_str(), feedback->num_retries);
        };
    
    // Define callback for result
    send_goal_options.result_callback = 
        [this, goal_handle](const rclcpp_action::ClientGoalHandle<DockRobot>::WrappedResult& wrapped_result) {
            
            auto result = std::make_shared<Dock::Result>();
            result->success = wrapped_result.result->success;
            result->error_code = wrapped_result.result->error_code;
            result->num_retries = wrapped_result.result->num_retries;
            
            switch (wrapped_result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (result->success) {
                        RCLCPP_INFO(this->get_logger(), "Docking succeeded!");
                        goal_handle->succeed(result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Docking failed with error code: %d", result->error_code);
                        goal_handle->abort(result);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Docking aborted");
                    goal_handle->abort(result);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "Docking canceled");
                    goal_handle->canceled(result);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    goal_handle->abort(result);
                    break;
            }
        };
    
    // Send goal
    docking_client_->async_send_goal(dock_goal, send_goal_options);
}

} // namespace open_mower_next::docking

// Register the component with the component manager
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::docking::DockingActionServer)