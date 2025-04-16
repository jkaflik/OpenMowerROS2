#include "map_recorder/map_recorder_node.hpp"
#include "map_recorder/utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace open_mower_next::map_recorder
{
MapRecorderNode::MapRecorderNode(const rclcpp::NodeOptions& options)
  : Node("map_recorder", options)
  , is_recording_area_(false)
  , is_charging_detected_(false)
  , auto_recording_mode_(true)
  , distance_threshold_(0.5)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  record_docking_station_server_ = rclcpp_action::create_server<RecordDockingStationAction>(
      this, "record_docking_station", std::bind(&MapRecorderNode::handleDockingGoal, this, _1, _2),
      std::bind(&MapRecorderNode::handleDockingCancel, this, _1),
      std::bind(&MapRecorderNode::handleDockingAccepted, this, _1));

  record_area_boundary_server_ = rclcpp_action::create_server<RecordAreaBoundaryAction>(
      this, "record_area_boundary", std::bind(&MapRecorderNode::handleAreaBoundaryGoal, this, _1, _2),
      std::bind(&MapRecorderNode::handleAreaBoundaryCancel, this, _1),
      std::bind(&MapRecorderNode::handleAreaBoundaryAccepted, this, _1));

  save_docking_station_client_ = create_client<open_mower_next::srv::SaveDockingStation>("save_docking_station");
  save_area_client_ = create_client<open_mower_next::srv::SaveArea>("save_area");

  charging_status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/power/charger_present", 10, std::bind(&MapRecorderNode::chargingStatusCallback, this, _1));

  RCLCPP_INFO(get_logger(), "Map Recorder node initialized");
}

MapRecorderNode::~MapRecorderNode()
{
}

rclcpp_action::GoalResponse MapRecorderNode::handleDockingGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RecordDockingStationAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request for docking station recording: %s", goal->name.c_str());

  if (goal->name.empty())
  {
    RCLCPP_ERROR(get_logger(), "Docking station name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if ((docking_goal_handle_ && docking_goal_handle_->is_active()) || is_recording_area_)
  {
    RCLCPP_ERROR(get_logger(), "Another recording is already in progress");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MapRecorderNode::handleDockingCancel(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking station recording");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MapRecorderNode::handleDockingAccepted(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle)
{
  is_charging_detected_ = false;
  docking_goal_handle_ = goal_handle;

  std::thread{ [this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RecordDockingStationAction::Feedback>();
    auto result = std::make_shared<RecordDockingStationAction::Result>();

    RCLCPP_INFO(get_logger(), "Recording docking station: %s", goal->name.c_str());

    feedback->status = "Waiting for charging to be detected...";
    goal_handle->publish_feedback(feedback);

    // TODO:
    // ros2 action send_goal \
    //   /drive_on_heading \
    //   nav2_msgs/action/DriveOnHeading \
    //   "{target: {x: 2.0}, speed: 0.1, time_allowance: {sec: 30}}"
    // either we wait for goal execution is done or we get robot in charging position

    auto start_time = this->now();
    while (rclcpp::ok() && !is_charging_detected_ && (this->now() - start_time) < rclcpp::Duration(60s))
    {
      if (goal_handle->is_canceling())
      {
        result->success = false;
        result->message = "Docking station recording was canceled";
        goal_handle->canceled(result);
        return;
      }

      std::this_thread::sleep_for(100ms);
    }

    if (!is_charging_detected_)
    {
      result->success = false;
      result->message = "Timeout waiting for charging to be detected";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Charging detected! Verifying position accuracy...";
    goal_handle->publish_feedback(feedback);

    if (!checkPositionCovariance())
    {
      result->success = false;
      result->message = "Position covariance exceeds threshold (2cm)";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Position verified. Recording docking station...";
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PoseStamped robot_pose;
    try
    {
      robot_pose = getRobotPose();
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = "Failed to get robot pose: " + std::string(e.what());
      goal_handle->abort(result);
      return;
    }

    open_mower_next::msg::DockingStation docking_station;
    docking_station.id = utils::generateUniqueId();
    docking_station.name = goal->name;
    docking_station.pose.header = robot_pose.header;
    docking_station.pose.pose = robot_pose.pose;

    auto request = std::make_shared<open_mower_next::srv::SaveDockingStation::Request>();
    request->docking_station = docking_station;

    if (!save_docking_station_client_->wait_for_service(10s))
    {
      result->success = false;
      result->message = "Save docking station service not available";
      goal_handle->abort(result);
      return;
    }

    auto future = save_docking_station_client_->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready)
    {
      result->success = false;
      result->message = "Save docking station service timed out";
      goal_handle->abort(result);
      return;
    }

    auto service_response = future.get();
    if (service_response)
    {
      result->success = service_response->success;
      result->message = service_response->message;

      if (service_response->success)
      {
        RCLCPP_INFO(get_logger(), "Docking station recording completed successfully");
        goal_handle->succeed(result);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to save docking station: %s", service_response->message.c_str());
        goal_handle->abort(result);
      }
    }
    else
    {
      result->success = false;
      result->message = "Failed to save docking station: service call failed";
      goal_handle->abort(result);
    }
    return;
  } }.detach();
}

rclcpp_action::GoalResponse MapRecorderNode::handleAreaBoundaryGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RecordAreaBoundaryAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request for area boundary recording: %s", goal->name.c_str());

  if (goal->name.empty())
  {
    RCLCPP_ERROR(get_logger(), "Area name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->type > 2)
  {
    RCLCPP_ERROR(get_logger(), "Invalid area type (must be 0, 1, or 2)");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (docking_goal_handle_ || is_recording_area_)
  {
    RCLCPP_ERROR(get_logger(), "Another recording is already in progress");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MapRecorderNode::handleAreaBoundaryCancel(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel area boundary recording");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MapRecorderNode::handleAreaBoundaryAccepted(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle)
{
  current_boundary_points_.clear();
  is_recording_area_ = true;
  area_boundary_goal_handle_ = goal_handle;
  auto goal = goal_handle->get_goal();

  current_area_name_ = goal->name;
  current_area_type_ = goal->type;
  auto_recording_mode_ = goal->auto_recording;
  distance_threshold_ = goal->distance_threshold;

  set_recording_mode_service_ = create_service<std_srvs::srv::SetBool>(
      "set_recording_mode", std::bind(&MapRecorderNode::handleSetRecordingMode, this, _1, _2));

  add_boundary_point_service_ = create_service<std_srvs::srv::Trigger>(
      "add_boundary_point", std::bind(&MapRecorderNode::handleAddBoundaryPoint, this, _1, _2));

  finish_area_recording_service_ = create_service<std_srvs::srv::Trigger>(
      "finish_area_recording", std::bind(&MapRecorderNode::handleFinishAreaRecording, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Area recording services are now available");

  std::thread{ [this, goal_handle]() {
    auto feedback = std::make_shared<RecordAreaBoundaryAction::Feedback>();
    auto result = std::make_shared<RecordAreaBoundaryAction::Result>();

    RCLCPP_INFO(get_logger(), "Recording area boundary: %s (type: %u)", current_area_name_.c_str(), current_area_type_);

    try
    {
      last_recorded_position_ = getRobotPose();

      current_boundary_points_.push_back(last_recorded_position_.pose.position);

      geometry_msgs::msg::PolygonStamped area_poly;
      area_poly.header.frame_id = "map";
      area_poly.header.stamp = this->now();

      geometry_msgs::msg::Point32 pt;
      pt.x = last_recorded_position_.pose.position.x;
      pt.y = last_recorded_position_.pose.position.y;
      pt.z = last_recorded_position_.pose.position.z;
      area_poly.polygon.points.push_back(pt);

      feedback->status = "Recording started. Added first boundary point.";
      feedback->point_count = 1;
      feedback->area = area_poly;
      goal_handle->publish_feedback(feedback);

      while (rclcpp::ok() && is_recording_area_)
      {
        if (goal_handle->is_canceling())
        {
          result->success = false;
          result->message = "Area boundary recording was canceled";
          goal_handle->canceled(result);
          is_recording_area_ = false;

          set_recording_mode_service_.reset();
          add_boundary_point_service_.reset();
          finish_area_recording_service_.reset();
          RCLCPP_INFO(get_logger(), "Area recording services have been removed");

          return;
        }

        if (auto_recording_mode_)
        {
          geometry_msgs::msg::PoseStamped current_pose = getRobotPose();
          double dist = utils::poseDistance(current_pose, last_recorded_position_);

          if (dist > distance_threshold_)
          {
            current_boundary_points_.push_back(current_pose.pose.position);
            last_recorded_position_ = current_pose;

            geometry_msgs::msg::PolygonStamped updated_poly;
            updated_poly.header.frame_id = "map";
            updated_poly.header.stamp = this->now();

            for (const auto& point : current_boundary_points_)
            {
              geometry_msgs::msg::Point32 p;
              p.x = point.x;
              p.y = point.y;
              p.z = point.z;
              updated_poly.polygon.points.push_back(p);
            }

            feedback->status = "Added point automatically";
            feedback->point_count = current_boundary_points_.size();
            feedback->area = updated_poly;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(get_logger(), "Added boundary point #%zu automatically", current_boundary_points_.size());
          }
        }

        std::this_thread::sleep_for(100ms);
      }

      set_recording_mode_service_.reset();
      add_boundary_point_service_.reset();
      finish_area_recording_service_.reset();
      RCLCPP_INFO(get_logger(), "Area recording services have been removed");

      if (current_boundary_points_.size() < 3)
      {
        result->success = false;
        result->message = "Not enough points for a valid area boundary (minimum 3)";
        goal_handle->abort(result);
        return;
      }

      if (!utils::isValidPolygon(current_boundary_points_))
      {
        result->success = false;
        result->message = "Invalid polygon formation";
        goal_handle->abort(result);
        return;
      }

      open_mower_next::msg::Area area;
      area.id = utils::generateUniqueId();
      area.name = current_area_name_;
      area.type = current_area_type_;

      geometry_msgs::msg::PolygonStamped polygon;
      polygon.header.frame_id = "map";
      polygon.header.stamp = this->now();

      for (const auto& point : current_boundary_points_)
      {
        geometry_msgs::msg::Point32 pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        polygon.polygon.points.push_back(pt);
      }
      area.area = polygon;

      auto request = std::make_shared<open_mower_next::srv::SaveArea::Request>();
      request->area = area;

      if (!save_area_client_->wait_for_service(10s))
      {
        result->success = false;
        result->message = "Save area service not available";
        goal_handle->abort(result);
        return;
      }

      auto future = save_area_client_->async_send_request(request);

      if (future.wait_for(5s) != std::future_status::ready)
      {
        result->success = false;
        result->message = "Save area service timed out";
        goal_handle->abort(result);
        return;
      }

      auto service_response = future.get();
      if (service_response)
      {
        result->success = service_response->success;
        result->message = service_response->message;
        result->area = area;

        if (service_response->success)
        {
          RCLCPP_INFO(get_logger(), "Area boundary recording completed successfully");
          goal_handle->succeed(result);
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Failed to save area: %s", service_response->message.c_str());
          goal_handle->abort(result);
        }
      }
      else
      {
        result->success = false;
        result->message = "Failed to save area, service call failed";
        goal_handle->abort(result);
      }
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = std::string("Error during recording: ") + e.what();
      goal_handle->abort(result);
      is_recording_area_ = false;

      set_recording_mode_service_.reset();
      add_boundary_point_service_.reset();
      finish_area_recording_service_.reset();
      RCLCPP_INFO(get_logger(), "Area recording services have been removed");
    }
  } }.detach();
}

void MapRecorderNode::handleSetRecordingMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  auto_recording_mode_ = request->data;

  response->success = true;
  response->message =
      auto_recording_mode_ ? "Switched to automatic recording mode" : "Switched to manual recording mode";

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MapRecorderNode::handleAddBoundaryPoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  if (auto_recording_mode_)
  {
    response->success = false;
    response->message = "Cannot manually add points in automatic mode";
    return;
  }

  try
  {
    geometry_msgs::msg::PoseStamped current_pose = getRobotPose();
    current_boundary_points_.push_back(current_pose.pose.position);
    last_recorded_position_ = current_pose;

    response->success = true;
    response->message = "Boundary point added successfully";

    RCLCPP_INFO(get_logger(), "Added boundary point #%zu manually", current_boundary_points_.size());

    if (area_boundary_goal_handle_)
    {
      auto feedback = std::make_shared<RecordAreaBoundaryAction::Feedback>();
      feedback->status = "Added point manually";
      feedback->point_count = current_boundary_points_.size();

      geometry_msgs::msg::PolygonStamped updated_poly;
      updated_poly.header.frame_id = "map";
      updated_poly.header.stamp = this->now();

      for (const auto& point : current_boundary_points_)
      {
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        updated_poly.polygon.points.push_back(p);
      }

      feedback->area = updated_poly;
      area_boundary_goal_handle_->publish_feedback(feedback);
    }
  }
  catch (const std::exception& e)
  {
    response->success = false;
    response->message = std::string("Failed to add boundary point: ") + e.what();
  }
}

void MapRecorderNode::handleFinishAreaRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  if (!area_boundary_goal_handle_)
  {
    response->success = false;
    response->message = "No area boundary goal handle available";
    return;
  }

  is_recording_area_ = false;
  response->success = true;
  response->message = "Area recording finished. Processing and saving...";

  RCLCPP_INFO(get_logger(), "Finalizing area boundary recording with %zu points", current_boundary_points_.size());

  return;
}

void MapRecorderNode::chargingStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_charging_detected_ = msg->data;
}

geometry_msgs::msg::PoseStamped MapRecorderNode::getRobotPose()
{
  geometry_msgs::msg::PoseStamped robot_pose;
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform("map", "charging_port", tf2::TimePointZero);

    // apply 180-degree rotation to get docking station face in front of robot's charging port
    tf2::Quaternion q;
    tf2::fromMsg(transform_stamped.transform.rotation, q);
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, M_PI);  // 180 degrees around Z axis
    q = q * rotation;

    robot_pose.header = transform_stamped.header;
    robot_pose.pose.position.x = transform_stamped.transform.translation.x;
    robot_pose.pose.position.y = transform_stamped.transform.translation.y;
    robot_pose.pose.position.z = transform_stamped.transform.translation.z;
    robot_pose.pose.orientation = tf2::toMsg(q);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not transform from base_link to map: %s", ex.what());
    throw;
  }

  return robot_pose;
}

bool MapRecorderNode::checkPositionCovariance()
{
  // TODO: check position
  return true;
}

std::string MapRecorderNode::generateUniqueId()
{
  return utils::generateUniqueId();
}

double MapRecorderNode::distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                                             const geometry_msgs::msg::PoseStamped& pose2)
{
  return utils::poseDistance(pose1, pose2);
}

bool MapRecorderNode::validateAreaBoundary()
{
  return utils::isValidPolygon(current_boundary_points_);
}
}  // namespace open_mower_next::map_recorder
