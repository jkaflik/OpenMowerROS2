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
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import threading
import signal
import sys
import os
import numpy as np

class OpenMowerController(Node):
    """ROS2 node for controlling OpenMower via terminal UI"""

    def __init__(self):
        super().__init__('openmower_terminal_controller')
        
        # Create publishers
        self.cmd_pub = self.create_publisher(
            TwistStamped, 
            '/cmd_vel_joy', 
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
        self.battery_state['percentage'] = msg.percentage
    
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
        
        # Create windows
        header_win = curses.newwin(3, curses.COLS, 0, 0)
        status_win = curses.newwin(8, curses.COLS // 2, 3, 0)
        control_win = curses.newwin(8, curses.COLS // 2, 3, curses.COLS // 2)
        help_win = curses.newwin(5, curses.COLS, 11, 0)
        
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
            
            # Update movement based on current active keys
            self._update_movement()
            
            # Clear windows
            header_win.clear()
            status_win.clear()
            control_win.clear()
            help_win.clear()
            
            # Draw header
            header_win.addstr(0, 0, "OpenMower Terminal Controller", curses.A_BOLD | curses.color_pair(5))
            header_win.addstr(1, 0, "=" * (curses.COLS - 1))
            
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
            help_win.addstr(3, 2, "Hold keys for continuous movement, release to stop", curses.color_pair(2))
            
            # Refresh windows
            header_win.refresh()
            status_win.refresh()
            control_win.refresh()
            help_win.refresh()
            
            # Publish current speeds
            self.publish_cmd_vel()
            
            # Sleep for update rate - shorter sleep for more responsive controls
            time.sleep(self.update_rate)
    
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