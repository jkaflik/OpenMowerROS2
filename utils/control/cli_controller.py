#!/usr/bin/env python3
"""
OpenMower Terminal Controller

A terminal-based UI for controlling the OpenMower robot and displaying its status.
Use arrow keys to navigate and control the robot's movement.
"""

import curses
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import threading
import signal
import sys
import os
import numpy as np
from enum import Enum, auto

# Import action messages
from nav2_msgs.action import DriveOnHeading, Spin
from open_mower_next.action import DockRobotNearest, DockRobotTo

class UIState(Enum):
    """Enum representing the state of the UI"""
    MAIN = auto()
    DIALOG = auto()
    ACTION_STATUS = auto()

class DialogResult:
    """Class representing the result of a dialog"""
    def __init__(self, success=False, values=None):
        self.success = success
        self.values = values or {}

class Dialog:
    """Class representing a dialog window with fields and buttons"""
    def __init__(self, title, fields, max_width=60, max_height=20):
        self.title = title
        self.fields = fields  # List of (label, default_value, field_type) tuples
        self.max_width = max_width
        self.max_height = max_height
        self.active_field = 0
        self.field_values = [field[1] for field in fields]
        self.button_selected = False  # False for OK, True for Cancel
    
    def show(self, stdscr):
        """Show the dialog and handle input"""
        height, width = stdscr.getmaxyx()
        
        # Calculate dimensions and position
        dialog_height = len(self.fields) + 6  # Fields + title + buttons + borders
        dialog_width = min(self.max_width, width - 4)
        start_y = (height - dialog_height) // 2
        start_x = (width - dialog_width) // 2
        
        # Create dialog window
        dialog_win = curses.newwin(dialog_height, dialog_width, start_y, start_x)
        dialog_win.keypad(True)
        dialog_win.border()
        
        # Draw title
        dialog_win.addstr(0, (dialog_width - len(self.title) - 2) // 2, f" {self.title} ")
        
        # Draw fields
        for i, (label, _, _) in enumerate(self.fields):
            # Display label
            dialog_win.addstr(i + 2, 2, f"{label}: ")
            
        # Draw buttons
        btn_y = dialog_height - 2
        ok_x = dialog_width // 3 - 2
        cancel_x = (2 * dialog_width) // 3 - 3
        
        # Main input loop
        while True:
            # Display current field values
            for i, value in enumerate(self.field_values):
                field_start_x = len(self.fields[i][0]) + 4
                # Clear field space
                dialog_win.addstr(i + 2, field_start_x, " " * (dialog_width - field_start_x - 3))
                # Display current value
                dialog_win.addstr(i + 2, field_start_x, str(value))
            
            # Display buttons
            if not self.button_selected:
                dialog_win.addstr(btn_y, ok_x, "[OK]", curses.A_REVERSE)
                dialog_win.addstr(btn_y, cancel_x, "[Cancel]")
            else:
                dialog_win.addstr(btn_y, ok_x, "[OK]")
                dialog_win.addstr(btn_y, cancel_x, "[Cancel]", curses.A_REVERSE)
            
            # Highlight current field if not on buttons
            if not self.button_selected:
                field_y = self.active_field + 2
                field_x = len(self.fields[self.active_field][0]) + 4
                dialog_win.chgat(field_y, field_x, len(str(self.field_values[self.active_field])), curses.A_REVERSE)
            
            dialog_win.refresh()
            
            # Get input
            key = dialog_win.getch()
            
            # Handle navigation keys
            if key == curses.KEY_UP:
                if self.button_selected:
                    self.button_selected = False
                    self.active_field = len(self.fields) - 1
                elif self.active_field > 0:
                    self.active_field -= 1
            elif key == curses.KEY_DOWN:
                if self.active_field == len(self.fields) - 1:
                    self.button_selected = True
                elif not self.button_selected:
                    self.active_field += 1
            elif key == curses.KEY_LEFT and self.button_selected:
                self.button_selected = False
            elif key == curses.KEY_RIGHT and self.button_selected:
                self.button_selected = True
            # Handle field editing
            elif key == curses.KEY_BACKSPACE or key == 127:  # Handle backspace
                if not self.button_selected:
                    value = str(self.field_values[self.active_field])
                    if value:
                        self.field_values[self.active_field] = value[:-1]
            # Handle button selection
            elif key == 10 or key == 13:  # Enter key
                if self.button_selected:
                    return DialogResult(not self.button_selected, {})
                else:
                    # Check if we're on a button
                    if self.active_field >= len(self.fields):
                        return DialogResult(not self.button_selected, {})
                    else:
                        # Convert field values based on their types
                        result = {}
                        for i, (label, _, field_type) in enumerate(self.fields):
                            try:
                                if field_type == float:
                                    result[label] = float(self.field_values[i])
                                elif field_type == int:
                                    result[label] = int(self.field_values[i])
                                elif field_type == bool:
                                    value = str(self.field_values[i]).lower()
                                    result[label] = value in ('true', 'yes', 'y', '1')
                                else:
                                    result[label] = self.field_values[i]
                            except ValueError:
                                # If conversion fails, use the default
                                if field_type == float:
                                    result[label] = 0.0
                                elif field_type == int:
                                    result[label] = 0
                                elif field_type == bool:
                                    result[label] = False
                                else:
                                    result[label] = ""
                        return DialogResult(True, result)
            elif key == 27:  # Escape key
                return DialogResult(False, {})
            # Handle regular input for text fields
            elif not self.button_selected and key != -1:
                value = str(self.field_values[self.active_field])
                field_type = self.fields[self.active_field][2]
                
                # Handle numeric fields
                if field_type in (int, float):
                    if (chr(key).isdigit() or 
                        (key == ord('.') and field_type == float and '.' not in value) or
                        (key == ord('-') and not value)):
                        self.field_values[self.active_field] = value + chr(key)
                else:
                    # For string fields, allow any printable character
                    if key >= 32 and key <= 126:  # printable ASCII
                        self.field_values[self.active_field] = value + chr(key)

class ActionGoalTracker:
    """Class for tracking action goals and their status"""
    def __init__(self, action_type, action_client, goal_msg, feedback_callback=None, result_callback=None):
        self.action_type = action_type
        self.action_name = action_type.__name__
        self.action_client = action_client
        self.goal_msg = goal_msg
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback
        self.goal_handle = None
        self.status = GoalStatus.STATUS_UNKNOWN
        self.feedback = None
        self.result = None
        self.is_active = True
        self.start_time = time.time()
    
    def cancel(self):
        """Cancel the goal if active"""
        if self.goal_handle and self.is_active:
            self.action_client.cancel_goal_async(self.goal_handle)


class OpenMowerController(Node):
    """ROS2 node for controlling OpenMower via terminal UI"""

    def __init__(self):
        super().__init__('openmower_terminal_controller')
        
        # Create publishers
        self.cmd_pub = self.create_publisher(
            TwistStamped, 
            '/cmd_joy', 
            10
        )
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/odometry/filtered/map',
            self.odom_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/power',
            self.battery_callback,
            10
        )
        
        self.charger_sub = self.create_subscription(
            Bool,
            '/power/charger_present',
            self.charger_callback,
            10
        )
        
        # Create action clients
        self.drive_on_heading_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.dock_robot_nearest_client = ActionClient(self, DockRobotNearest, 'dock_robot_nearest')
        self.dock_robot_to_client = ActionClient(self, DockRobotTo, 'dock_robot_to')
        
        # Initialize state variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0  # Simplified to yaw only, in degrees relative to NORTH
        self.battery_state = {
            'voltage': 0.0,
            'current': 0.0,
            'percentage': 0.0,
        }
        self.is_charging = False
        
        # Control parameters
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.linear_speed_increment = 0.1  # Speed increment (m/s)
        
        # Active keys tracking
        self.active_keys = {
            curses.KEY_UP: False,
            curses.KEY_DOWN: False,
            curses.KEY_LEFT: False,
            curses.KEY_RIGHT: False
        }
        
        # Combination key tracking (to handle multiple keys at once)
        self.combination_keys = {
            'up_left': False,
            'up_right': False,
            'down_left': False,
            'down_right': False
        }
        
        # Key press timing to handle repeats
        self.key_timestamps = {key: 0 for key in self.active_keys}
        self.key_timeout = 0.15  # seconds until a key is considered released
        
        # UI state tracking
        self.ui_state = UIState.MAIN
        self.current_dialog = None
        
        # Action tracking
        self.current_action = None
        
        # UI update rate
        self.update_rate = 0.02  # faster update rate for more responsive controls
        
        # Setup curses UI in a separate thread
        self.ui_thread = threading.Thread(target=self._run_ui)
        self.ui_thread.daemon = True
        self.running = True
        self.should_exit = False
        
        # Setup signal handling for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.running = False
        self.should_exit = True
        self.get_logger().info('Shutting down...')
        # Force exit after a short delay if normal exit doesn't work
        threading.Timer(0.5, lambda: os._exit(0)).start()
        
    def odom_callback(self, msg):
        """Process odometry data"""
        # Extract position (2D only)
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract yaw only from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Calculate yaw (relative to NORTH = 0 degrees)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees and normalize to 0-360 range (NORTH = 0)
        yaw_deg = math.degrees(yaw_rad)
        self.yaw = (yaw_deg + 360) % 360
    
    def battery_callback(self, msg):
        """Process battery state data"""
        self.battery_state['voltage'] = msg.voltage
        self.battery_state['current'] = msg.current
        self.battery_state['percentage'] = msg.percentage * 100.0  # Convert to percentage
    
    def charger_callback(self, msg):
        """Process charger presence data"""
        self.is_charging = msg.data
    
    def publish_cmd_vel(self):
        """Publish velocity commands to the robot"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Set linear and angular velocities
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = self.angular_speed
        
        self.cmd_pub.publish(msg)
    
    def _create_drive_on_heading_dialog(self):
        """Create dialog for DriveOnHeading action"""
        fields = [
            ("Target Distance (m)", "1.0", float),
            ("Speed (m/s)", "0.2", float),
            ("Time Allowance (s)", "10", int)
        ]
        return Dialog("Drive on Heading", fields)
    
    def _create_spin_dialog(self):
        """Create dialog for Spin action"""
        fields = [
            ("Target Angle (deg)", "90", float),
            ("Angular Speed (rad/s)", "0.5", float),
            ("Time Allowance (s)", "10", int)
        ]
        return Dialog("Spin Action", fields)
    
    def _create_dock_robot_to_dialog(self):
        """Create dialog for DockRobotTo action"""
        fields = [
            ("Docking Station ID", "", str),
        ]
        return Dialog("Dock to Specific Station", fields)
    
    def _send_drive_on_heading_goal(self, values):
        """Send a DriveOnHeading action goal"""
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = values.get("Target Distance (m)", 1.0)
        goal_msg.speed = values.get("Speed (m/s)", 0.2)
        
        # Set time allowance
        time_allowance = values.get("Time Allowance (s)", 10)
        goal_msg.time_allowance.sec = time_allowance
        goal_msg.time_allowance.nanosec = 0
        
        self._send_action_goal(
            self.drive_on_heading_client, 
            goal_msg, 
            "Drive on Heading", 
            self._drive_on_heading_feedback_callback,
            self._drive_on_heading_result_callback
        )
    
    def _send_spin_goal(self, values):
        """Send a Spin action goal"""
        goal_msg = Spin.Goal()
        # Convert degrees to radians
        goal_msg.target_yaw = math.radians(values.get("Target Angle (deg)", 90.0))
        goal_msg.angular_velocity = values.get("Angular Speed (rad/s)", 0.5)
        
        # Set time allowance
        time_allowance = values.get("Time Allowance (s)", 10)
        goal_msg.time_allowance.sec = time_allowance
        goal_msg.time_allowance.nanosec = 0
        
        self._send_action_goal(
            self.spin_client, 
            goal_msg, 
            "Spin", 
            self._spin_feedback_callback,
            self._spin_result_callback
        )
    
    def _send_dock_robot_nearest_goal(self):
        """Send a DockRobotNearest action goal"""
        goal_msg = DockRobotNearest.Goal()
        
        self._send_action_goal(
            self.dock_robot_nearest_client, 
            goal_msg, 
            "Dock to Nearest", 
            self._dock_robot_nearest_feedback_callback,
            self._dock_robot_nearest_result_callback
        )
    
    def _send_dock_robot_to_goal(self, values):
        """Send a DockRobotTo action goal"""
        goal_msg = DockRobotTo.Goal()
        goal_msg.dock_id = values.get("Docking Station ID", "")
        
        self._send_action_goal(
            self.dock_robot_to_client, 
            goal_msg, 
            "Dock to Specific", 
            self._dock_robot_to_feedback_callback,
            self._dock_robot_to_result_callback
        )
    
    def _send_action_goal(self, client, goal_msg, action_name, feedback_cb, result_cb):
        """Send an action goal and set up tracking"""
        action_type = type(goal_msg)
        
        # Initialize action tracker
        self.current_action = ActionGoalTracker(
            action_type=action_type,
            action_client=client,
            goal_msg=goal_msg,
            feedback_callback=feedback_cb,
            result_callback=result_cb
        )
        
        # Send the goal
        self.get_logger().info(f'Sending {action_name} goal')
        client.wait_for_server()
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_cb
        )
        
        # Set callback for goal response
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        # Set UI to action status mode
        self.ui_state = UIState.ACTION_STATUS
    
    def _goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            if self.current_action:
                self.current_action.is_active = False
            return
        
        if self.current_action:
            self.current_action.goal_handle = goal_handle
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        status = future.result().status
        
        if self.current_action:
            self.current_action.result = result
            self.current_action.status = status
            self.current_action.is_active = False
            
            if self.current_action.result_callback:
                self.current_action.result_callback(result, status)
    
    # Feedback and result callbacks for specific actions
    def _drive_on_heading_feedback_callback(self, feedback_msg):
        """Process DriveOnHeading action feedback"""
        feedback = feedback_msg.feedback
        if self.current_action:
            self.current_action.feedback = feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
    
    def _drive_on_heading_result_callback(self, result, status):
        """Process DriveOnHeading action result"""
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Drive on heading completed successfully')
        else:
            self.get_logger().info(f'Drive on heading failed with status: {status}')
    
    def _spin_feedback_callback(self, feedback_msg):
        """Process Spin action feedback"""
        feedback = feedback_msg.feedback
        if self.current_action:
            self.current_action.feedback = feedback
        self.get_logger().info(f'Angular distance remaining: {feedback.angular_distance_traveled}')
    
    def _spin_result_callback(self, result, status):
        """Process Spin action result"""
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Spin completed successfully')
        else:
            self.get_logger().info(f'Spin failed with status: {status}')
    
    def _dock_robot_nearest_feedback_callback(self, feedback_msg):
        """Process DockRobotNearest action feedback"""
        feedback = feedback_msg.feedback
        if self.current_action:
            self.current_action.feedback = feedback
        self.get_logger().info(f'Status: {feedback.status}, Message: {feedback.message}')
    
    def _dock_robot_nearest_result_callback(self, result, status):
        """Process DockRobotNearest action result"""
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Dock to nearest completed successfully')
        else:
            self.get_logger().info(f'Dock to nearest failed with status: {status}')
    
    def _dock_robot_to_feedback_callback(self, feedback_msg):
        """Process DockRobotTo action feedback"""
        feedback = feedback_msg.feedback
        if self.current_action:
            self.current_action.feedback = feedback
        self.get_logger().info(f'Status: {feedback.status}, Message: {feedback.message}')
    
    def _dock_robot_to_result_callback(self, result, status):
        """Process DockRobotTo action result"""
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Dock to specific completed successfully')
        else:
            self.get_logger().info(f'Dock to specific failed with status: {status}')
    
    def _run_ui(self):
        """Run the terminal UI"""
        try:
            curses.wrapper(self._curses_main)
        except Exception as e:
            self.get_logger().error(f"UI error: {str(e)}")
        finally:
            # Ensure clean exit when UI thread ends
            if self.should_exit:
                os._exit(0)
    
    def _curses_main(self, stdscr):
        """Main curses application"""
        # Setup curses
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(1)  # Make getch non-blocking
        curses.start_color()
        curses.use_default_colors()
        curses.raw()  # Raw mode for better key handling
        
        # Enable special keys
        stdscr.keypad(True)
        
        # Define color pairs
        curses.init_pair(1, curses.COLOR_GREEN, -1)  # Good status (green)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)  # Warning status (yellow)
        curses.init_pair(3, curses.COLOR_RED, -1)  # Critical status (red)
        curses.init_pair(4, curses.COLOR_CYAN, -1)  # Info (cyan)
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)  # Special (magenta)
        
        # Reset all active keys at the start
        for key in self.active_keys:
            self.active_keys[key] = False
        for combo in self.combination_keys:
            self.combination_keys[combo] = False
            
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            elapsed = current_time - last_time
            last_time = current_time
            
            # Handle input based on current UI state
            if self.ui_state == UIState.MAIN:
                self._handle_main_input(stdscr, current_time)
            elif self.ui_state == UIState.DIALOG and self.current_dialog:
                # Dialogs handle their own input
                result = self.current_dialog.show(stdscr)
                if result.success:
                    # Process the dialog result
                    if isinstance(self.current_dialog, self._create_drive_on_heading_dialog().__class__) and self.current_dialog.title == "Drive on Heading":
                        self._send_drive_on_heading_goal(result.values)
                    elif isinstance(self.current_dialog, self._create_spin_dialog().__class__) and self.current_dialog.title == "Spin Action":
                        self._send_spin_goal(result.values)
                    elif isinstance(self.current_dialog, self._create_dock_robot_to_dialog().__class__) and self.current_dialog.title == "Dock to Specific Station":
                        self._send_dock_robot_to_goal(result.values)
                
                # Return to main UI
                self.ui_state = UIState.MAIN if self.ui_state != UIState.ACTION_STATUS else UIState.ACTION_STATUS
                self.current_dialog = None
                
                # Redraw the screen
                stdscr.clear()
                stdscr.refresh()
            elif self.ui_state == UIState.ACTION_STATUS:
                self._handle_action_status_input(stdscr)
            
            # Update combination keys based on active keys
            self._update_combination_keys()
            
            # Update movement based on active keys
            if self.ui_state == UIState.MAIN:
                self._update_movement()
            else:
                # Stop movement in other UI states
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            
            # Render the appropriate UI based on current state
            if self.ui_state == UIState.MAIN:
                self._render_main_ui(stdscr)
            elif self.ui_state == UIState.ACTION_STATUS:
                self._render_action_status_ui(stdscr)
            
            # Publish current speeds (only in main state)
            if self.ui_state == UIState.MAIN:
                self.publish_cmd_vel()
            
            # Sleep for update rate - shorter sleep for responsive controls
            time.sleep(self.update_rate)
    
    def _handle_main_input(self, stdscr, current_time):
        """Handle input in the main UI state"""
        # Store previous key states for combination detection
        prev_key_states = self.active_keys.copy()
        
        # Expire keys that haven't been pressed recently
        for key in self.active_keys:
            if current_time - self.key_timestamps[key] > self.key_timeout:
                self.active_keys[key] = False
        
        # Check for key presses - increased number of checks to catch more key events
        max_key_checks = 15  # Check more times per cycle to catch multiple key presses
        for _ in range(max_key_checks):
            try:
                key = stdscr.getch()
                if key != -1:
                    # Check if this is a valid key we're tracking
                    if key in self.active_keys:
                        self.active_keys[key] = True
                        self.key_timestamps[key] = current_time  # Update timestamp
                    self._handle_key(key)
                else:
                    break
            except Exception:
                break
    
    def _handle_action_status_input(self, stdscr):
        """Handle input in the action status UI state"""
        try:
            key = stdscr.getch()
            if key != -1:
                if key == ord('c') or key == ord('C'):  # Cancel action
                    if self.current_action and self.current_action.is_active:
                        self.current_action.cancel()
                elif key == ord('b') or key == ord('B'):  # Back to main UI
                    if self.current_action and not self.current_action.is_active:
                        self.ui_state = UIState.MAIN
                        self.current_action = None
                elif key == ord('q') or key == ord('Q'):  # Quit
                    self.running = False
                    self.should_exit = True
                    threading.Timer(0.1, lambda: os._exit(0)).start()
        except Exception:
            pass
    
    def _update_combination_keys(self):
        """Update combination key states based on active keys"""
        # Detect combination key presses
        if self.active_keys[curses.KEY_UP] and self.active_keys[curses.KEY_LEFT]:
            self.combination_keys['up_left'] = True
        else:
            self.combination_keys['up_left'] = False
            
        if self.active_keys[curses.KEY_UP] and self.active_keys[curses.KEY_RIGHT]:
            self.combination_keys['up_right'] = True
        else:
            self.combination_keys['up_right'] = False
            
        if self.active_keys[curses.KEY_DOWN] and self.active_keys[curses.KEY_LEFT]:
            self.combination_keys['down_left'] = True
        else:
            self.combination_keys['down_left'] = False
            
        if self.active_keys[curses.KEY_DOWN] and self.active_keys[curses.KEY_RIGHT]:
            self.combination_keys['down_right'] = True
        else:
            self.combination_keys['down_right'] = False
    
    def _handle_key(self, key):
        """Handle keyboard input"""
        if key == ord(' '):  # Space - stop
            for k in self.active_keys:
                self.active_keys[k] = False
        elif key == ord('q') or key == ord('Q'):  # Q - quit
            self.running = False
            self.should_exit = True
            # Force exit after a short delay if normal exit doesn't work
            threading.Timer(0.1, lambda: os._exit(0)).start()
        elif key == ord('+') or key == ord('='):  # Increase max speed
            self.max_linear_speed = min(self.max_linear_speed + self.linear_speed_increment, 1.0)
        elif key == ord('-') or key == ord('_'):  # Decrease max speed
            self.max_linear_speed = max(self.max_linear_speed - self.linear_speed_increment, 0.1)
        elif key == 3:  # Ctrl+C
            self.running = False
            self.should_exit = True
            # Force exit immediately
            os._exit(0)
        elif key == ord('d') or key == ord('D'):  # Drive on heading
            self.current_dialog = self._create_drive_on_heading_dialog()
            self.ui_state = UIState.DIALOG
        elif key == ord('s') or key == ord('S'):  # Spin
            self.current_dialog = self._create_spin_dialog()
            self.ui_state = UIState.DIALOG
        elif key == ord('n') or key == ord('N'):  # Dock to nearest
            self._send_dock_robot_nearest_goal()
        elif key == ord('t') or key == ord('T'):  # Dock to specific
            self.current_dialog = self._create_dock_robot_to_dialog()
            self.ui_state = UIState.DIALOG
    
    def _update_movement(self):
        """Update movement based on current active keys and combinations"""
        # Reset speeds first
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # Handle combination keys for diagonal movement
        if self.combination_keys['up_left']:
            self.linear_speed = self.max_linear_speed
            self.angular_speed = self.max_angular_speed
            return
        elif self.combination_keys['up_right']:
            self.linear_speed = self.max_linear_speed
            self.angular_speed = -self.max_angular_speed
            return
        elif self.combination_keys['down_left']:
            self.linear_speed = -self.max_linear_speed
            self.angular_speed = self.max_angular_speed
            return
        elif self.combination_keys['down_right']:
            self.linear_speed = -self.max_linear_speed
            self.angular_speed = -self.max_angular_speed
            return
            
        # Set speeds based on active keys (when no combinations are active)
        if self.active_keys[curses.KEY_UP]:
            self.linear_speed = self.max_linear_speed
        elif self.active_keys[curses.KEY_DOWN]:
            self.linear_speed = -self.max_linear_speed
            
        if self.active_keys[curses.KEY_LEFT]:
            self.angular_speed = self.max_angular_speed
        elif self.active_keys[curses.KEY_RIGHT]:
            self.angular_speed = -self.max_angular_speed
    
    def _render_main_ui(self, stdscr):
        """Render the main UI"""
        height, width = stdscr.getmaxyx()
        
        # Clear screen
        stdscr.clear()
        
        # Create windows
        header_win = curses.newwin(3, width, 0, 0)
        status_win = curses.newwin(8, width // 2, 3, 0)
        control_win = curses.newwin(8, width // 2, 3, width // 2)
        help_win = curses.newwin(6, width, 11, 0)
        
        # Draw header
        header_win.addstr(0, 0, "OpenMower Terminal Controller", curses.A_BOLD | curses.color_pair(5))
        header_win.addstr(1, 0, "=" * (width - 1))
        
        # Draw status information
        status_win.addstr(0, 0, "Robot Status:", curses.A_BOLD)
        
        # Position and orientation information (single line)
        status_win.addstr(1, 2, "Position/Heading: ")
        pos_str = f"X: {self.position['x']:.2f}m, Y: {self.position['y']:.2f}m, Heading: {self.yaw:.1f}°"
        status_win.addstr(pos_str, curses.color_pair(4))
        
        # Battery and charging status (single line)
        status_win.addstr(2, 2, "Battery: ")
        
        # Determine battery percentage color
        if self.battery_state['percentage'] > 50:
            batt_color = curses.color_pair(1)  # Green for good
        elif self.battery_state['percentage'] > 20:
            batt_color = curses.color_pair(2)  # Yellow for warning
        else:
            batt_color = curses.color_pair(3)  # Red for critical
            
        batt_str = f"{self.battery_state['percentage']:.1f}% "
        batt_str += f"({self.battery_state['voltage']:.2f}V, {self.battery_state['current']:.2f}A)"
        
        # Add charging indicator
        if self.is_charging:
            batt_str += " [CHARGING]"
            
        status_win.addstr(batt_str, batt_color)
        
        # Control information
        control_win.addstr(0, 0, "Control:", curses.A_BOLD)
        control_win.addstr(1, 2, "Linear Speed: ")
        
        # Determine speed color
        linear_pct = abs(self.linear_speed / self.max_linear_speed) * 100
        if linear_pct < 30:
            speed_color = curses.color_pair(1)  # Green for slow
        elif linear_pct < 70:
            speed_color = curses.color_pair(2)  # Yellow for medium
        else:
            speed_color = curses.color_pair(3)  # Red for fast
            
        speed_str = f"{self.linear_speed:.2f} m/s (Max: {self.max_linear_speed:.2f} m/s)"
        control_win.addstr(speed_str, speed_color)
        
        control_win.addstr(2, 2, "Angular Speed: ")
        
        # Determine angular speed color
        angular_pct = abs(self.angular_speed / self.max_angular_speed) * 100
        if angular_pct < 30:
            turn_color = curses.color_pair(1)  # Green for slow
        elif angular_pct < 70:
            turn_color = curses.color_pair(2)  # Yellow for medium
        else:
            turn_color = curses.color_pair(3)  # Red for fast
            
        control_win.addstr(f"{self.angular_speed:.2f} rad/s", turn_color)
        
        # Key press indicators
        control_win.addstr(4, 2, "Active Keys:")
        key_status = ""
        if self.active_keys[curses.KEY_UP]:
            key_status += "↑ "
        if self.active_keys[curses.KEY_DOWN]:
            key_status += "↓ "
        if self.active_keys[curses.KEY_LEFT]:
            key_status += "← "
        if self.active_keys[curses.KEY_RIGHT]:
            key_status += "→ "
        
        # Show combination keys
        if any(self.combination_keys.values()):
            key_status += "("
            if self.combination_keys['up_left']:
                key_status += "↑+← "
            if self.combination_keys['up_right']:
                key_status += "↑+→ "
            if self.combination_keys['down_left']:
                key_status += "↓+← "
            if self.combination_keys['down_right']:
                key_status += "↓+→ "
            key_status += ")"
            
        if not any(self.active_keys.values()):
            key_status = "None"
        control_win.addstr(key_status, curses.color_pair(5))
        
        # Visual representation of current movement
        control_win.addstr(5, 2, "Movement Direction:")
        
        # Create a simple arrow diagram
        arrow_y, arrow_x = 6, 15
        if abs(self.linear_speed) < 0.01 and abs(self.angular_speed) < 0.01:
            control_win.addstr(arrow_y, arrow_x, "•", curses.A_BOLD)
        else:
            # Forward/backward arrows
            if self.linear_speed > 0.01:
                control_win.addstr(arrow_y - 1, arrow_x, "↑", curses.A_BOLD)
            elif self.linear_speed < -0.01:
                control_win.addstr(arrow_y + 1, arrow_x, "↓", curses.A_BOLD)
            
            # Left/right arrows
            if self.angular_speed > 0.01:
                control_win.addstr(arrow_y, arrow_x - 2, "↺", curses.A_BOLD)
            elif self.angular_speed < -0.01:
                control_win.addstr(arrow_y, arrow_x + 2, "↻", curses.A_BOLD)
            
            # Center dot if any movement
            control_win.addstr(arrow_y, arrow_x, "•")
        
        # Help text
        help_win.addstr(0, 0, "Controls:", curses.A_BOLD)
        help_win.addstr(1, 2, "↑/↓: Forward/Backward | ←/→: Turn Left/Right | Space: Stop | Q: Quit")
        help_win.addstr(2, 2, "+/-: Increase/Decrease max speed | Ctrl+C: Exit")
        help_win.addstr(3, 2, "D: Drive on Heading | S: Spin | N: Dock to Nearest | T: Dock to Specific")
        help_win.addstr(4, 2, "Hold keys for continuous movement, release to stop", curses.color_pair(2))
        
        # Refresh windows
        header_win.refresh()
        status_win.refresh()
        control_win.refresh()
        help_win.refresh()

    def _render_action_status_ui(self, stdscr):
        """Render the action status UI"""
        if not self.current_action:
            self.ui_state = UIState.MAIN
            return
            
        height, width = stdscr.getmaxyx()
        
        # Clear screen
        stdscr.clear()
        
        # Create windows
        header_win = curses.newwin(3, width, 0, 0)
        action_win = curses.newwin(height - 6, width, 3, 0)
        help_win = curses.newwin(3, width, height - 3, 0)
        
        # Draw header
        header_win.addstr(0, 0, f"Action Status: {self.current_action.action_name}", curses.A_BOLD | curses.color_pair(5))
        header_win.addstr(1, 0, "=" * (width - 1))
        
        # Draw action status information
        action_win.addstr(0, 0, "Action Details:", curses.A_BOLD)
        
        # Display time elapsed
        elapsed = time.time() - self.current_action.start_time
        action_win.addstr(1, 2, f"Time Elapsed: {elapsed:.1f} seconds")
        
        # Display goal status
        status_str = "Unknown"
        status_color = curses.color_pair(4)
        
        if self.current_action.status == GoalStatus.STATUS_EXECUTING:
            status_str = "Executing"
            status_color = curses.color_pair(2)
        elif self.current_action.status == GoalStatus.STATUS_SUCCEEDED:
            status_str = "Succeeded"
            status_color = curses.color_pair(1)
        elif self.current_action.status == GoalStatus.STATUS_CANCELED:
            status_str = "Canceled"
            status_color = curses.color_pair(2)
        elif self.current_action.status == GoalStatus.STATUS_ABORTED:
            status_str = "Aborted"
            status_color = curses.color_pair(3)
        
        action_win.addstr(2, 2, "Status: ")
        action_win.addstr(status_str, status_color)
        
        # Display feedback based on action type
        line = 4
        action_win.addstr(line, 0, "Feedback:", curses.A_BOLD)
        line += 1
        
        if self.current_action.feedback:
            if isinstance(self.current_action.feedback, DriveOnHeading.Feedback):
                action_win.addstr(line, 2, f"Distance Remaining: {self.current_action.feedback.distance_remaining:.2f} m")
                line += 1
                action_win.addstr(line, 2, f"Current Speed: {self.current_action.feedback.current_speed:.2f} m/s")
            elif isinstance(self.current_action.feedback, Spin.Feedback):
                action_win.addstr(line, 2, f"Angular Distance Traveled: {self.current_action.feedback.angular_distance_traveled:.2f} rad")
                line += 1
                action_win.addstr(line, 2, f"Remaining Angle: {self.current_action.feedback.remaining_angle_travel:.2f} rad")
            elif isinstance(self.current_action.feedback, (DockRobotNearest.Feedback, DockRobotTo.Feedback)):
                action_win.addstr(line, 2, f"Status: {self.current_action.feedback.status}")
                line += 1
                action_win.addstr(line, 2, f"Message: {self.current_action.feedback.message}")
                line += 1
                action_win.addstr(line, 2, f"Num Retries: {self.current_action.feedback.num_retries}")
                line += 1
                action_win.addstr(line, 2, f"Docking Time: {self.current_action.feedback.docking_time.sec}.{self.current_action.feedback.docking_time.nanosec//1000000:03d}s")
                line += 1
                if hasattr(self.current_action.feedback, 'chosen_docking_station'):
                    action_win.addstr(line, 2, f"Docking Station: {self.current_action.feedback.chosen_docking_station.name} (ID: {self.current_action.feedback.chosen_docking_station.id})")
        else:
            action_win.addstr(line, 2, "No feedback received yet")
            
        # Show results if action is complete
        if not self.current_action.is_active and self.current_action.result:
            line += 2
            action_win.addstr(line, 0, "Result:", curses.A_BOLD)
            line += 1
            
            if isinstance(self.current_action.result, (DockRobotNearest.Result, DockRobotTo.Result)):
                action_win.addstr(line, 2, f"Code: {self.current_action.result.code}")
                line += 1
                action_win.addstr(line, 2, f"Message: {self.current_action.result.message}")
                line += 1
                action_win.addstr(line, 2, f"Num Retries: {self.current_action.result.num_retries}")
            else:
                action_win.addstr(line, 2, f"Action completed with status: {self.current_action.status}")
        
        # Help text
        if self.current_action.is_active:
            help_win.addstr(0, 0, "Controls:", curses.A_BOLD)
            help_win.addstr(1, 2, "C: Cancel Action | Q: Quit", curses.color_pair(2))
        else:
            help_win.addstr(0, 0, "Controls:", curses.A_BOLD)
            help_win.addstr(1, 2, "B: Back to Main | Q: Quit", curses.color_pair(2))
        
        # Refresh windows
        header_win.refresh()
        action_win.refresh()
        help_win.refresh()
    
    def start(self):
        """Start the controller"""
        self.ui_thread.start()
        
def main(args=None):
    rclpy.init(args=args)
    controller = OpenMowerController()
    
    try:
        controller.start()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        # Ensure clean exit
        os._exit(0)

if __name__ == '__main__':
    main()