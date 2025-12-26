#!/usr/bin/env python3
"""
Swerve Drive Gamepad Controller
Maps Xbox 360 controller inputs to omnidirectional robot motion

Control Mapping:
- Left Stick X/Y: Translational velocity (vx, vy)
- Right Stick X: Rotational velocity (omega)
- LB/RB: Speed multiplier (25%, 50%, 100%)
- A Button: Emergency stop
- B Button: Reset to default speed
- Start: Toggle verbose logging

Author: Generated for testAMR7 swerve drive
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math


class SwerveGamepadController(Node):
    def __init__(self):
        super().__init__('swerve_gamepad_controller')
        
        # ====================================================================
        # PARAMETERS - All configurable via YAML
        # ====================================================================
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enable_turbo', True)
        self.declare_parameter('turbo_multiplier', 1.5)
        self.declare_parameter('enable_precision', True)
        self.declare_parameter('precision_multiplier', 0.25)
        
        # Axis mapping (standard Xbox 360)
        self.declare_parameter('axis_linear_x', 1)    # Left stick Y
        self.declare_parameter('axis_linear_y', 0)    # Left stick X
        self.declare_parameter('axis_angular', 3)     # Right stick X
        
        # Button mapping
        self.declare_parameter('button_emergency_stop', 0)  # A
        self.declare_parameter('button_reset_speed', 1)     # B
        self.declare_parameter('button_precision', 4)       # LB
        self.declare_parameter('button_turbo', 5)           # RB
        self.declare_parameter('button_toggle_verbose', 7)  # Start
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.deadzone = self.get_parameter('deadzone').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_turbo = self.get_parameter('enable_turbo').value
        self.turbo_mult = self.get_parameter('turbo_multiplier').value
        self.enable_precision = self.get_parameter('enable_precision').value
        self.precision_mult = self.get_parameter('precision_multiplier').value
        
        # Axis indices
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular = self.get_parameter('axis_angular').value
        
        # Button indices
        self.btn_estop = self.get_parameter('button_emergency_stop').value
        self.btn_reset = self.get_parameter('button_reset_speed').value
        self.btn_precision = self.get_parameter('button_precision').value
        self.btn_turbo = self.get_parameter('button_turbo').value
        self.btn_verbose = self.get_parameter('button_toggle_verbose').value
        
        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        self.current_twist = Twist()
        self.speed_multiplier = 1.0
        self.emergency_stopped = False
        self.verbose = False
        
        # Button state tracking (for edge detection)
        self.last_button_states = {}
        
        # ====================================================================
        # ROS INTERFACES
        # ====================================================================
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/omni_base_controller/cmd_vel',
            10
        )
        
        # Publish timer for smooth control
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        
        self.get_logger().info('=================================================')
        self.get_logger().info('Swerve Gamepad Controller Initialized')
        self.get_logger().info('=================================================')
        self.get_logger().info(f'Max Linear Velocity:  {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max Angular Velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'Deadzone:             {self.deadzone}')
        self.get_logger().info(f'Publish Rate:         {self.publish_rate} Hz')
        self.get_logger().info('=================================================')
        self.get_logger().info('CONTROLS:')
        self.get_logger().info('  Left Stick    → Translate (X/Y)')
        self.get_logger().info('  Right Stick X → Rotate')
        self.get_logger().info('  LB            → Precision Mode (25%)')
        self.get_logger().info('  RB            → Turbo Mode (150%)')
        self.get_logger().info('  A Button      → Emergency Stop')
        self.get_logger().info('  B Button      → Reset Speed')
        self.get_logger().info('  Start         → Toggle Verbose')
        self.get_logger().info('=================================================')

    def joy_callback(self, msg: Joy):
        """Process joystick input and update target velocities"""
        
        # ================================================================
        # BUTTON HANDLING (Edge Detection)
        # ================================================================
        self.handle_buttons(msg)
        
        # If emergency stopped, zero all velocities
        if self.emergency_stopped:
            self.current_twist.linear.x = 0.0
            self.current_twist.linear.y = 0.0
            self.current_twist.angular.z = 0.0
            return
        
        # ================================================================
        # READ AXES WITH SAFETY CHECKS
        # ================================================================
        axes = msg.axes
        
        # Left stick Y → Forward/Backward (vx in base_link frame)
        raw_vx = self.get_axis_safe(axes, self.axis_linear_x)
        
        # Left stick X → Left/Right strafe (vy in base_link frame)
        raw_vy = self.get_axis_safe(axes, self.axis_linear_y)
        
        # Right stick X → Rotation (omega)
        raw_omega = self.get_axis_safe(axes, self.axis_angular)
        
        # ================================================================
        # APPLY DEADZONE
        # ================================================================
        vx = self.apply_deadzone(raw_vx)
        vy = self.apply_deadzone(raw_vy)
        omega = self.apply_deadzone(raw_omega)
        
        # ================================================================
        # CHECK FOR SPEED MODIFIERS (LIVE BUTTONS)
        # ================================================================
        buttons = msg.buttons
        current_multiplier = 1.0
        
        # Precision mode (LB held)
        if self.enable_precision and self.get_button_safe(buttons, self.btn_precision):
            current_multiplier = self.precision_mult
        
        # Turbo mode (RB held) - overrides precision
        if self.enable_turbo and self.get_button_safe(buttons, self.btn_turbo):
            current_multiplier = self.turbo_mult
        
        self.speed_multiplier = current_multiplier
        
        # ================================================================
        # SCALE TO MAXIMUM VELOCITIES
        # ================================================================
        self.current_twist.linear.x = vx * self.max_linear_vel * self.speed_multiplier
        self.current_twist.linear.y = vy * self.max_linear_vel * self.speed_multiplier
        self.current_twist.angular.z = omega * self.max_angular_vel * self.speed_multiplier
        
        # ================================================================
        # VERBOSE LOGGING
        # ================================================================
        if self.verbose:
            self.get_logger().info(
                f'Raw: vx={raw_vx:.2f} vy={raw_vy:.2f} ω={raw_omega:.2f} | '
                f'Cmd: vx={self.current_twist.linear.x:.3f} '
                f'vy={self.current_twist.linear.y:.3f} '
                f'ω={self.current_twist.angular.z:.3f} | '
                f'Mult: {self.speed_multiplier:.2f}x'
            )

    def handle_buttons(self, msg: Joy):
        """Handle button presses with edge detection"""
        buttons = msg.buttons
        
        # Emergency Stop (A button) - PRESS
        if self.button_pressed(buttons, self.btn_estop):
            self.emergency_stopped = not self.emergency_stopped
            if self.emergency_stopped:
                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
                self.current_twist = Twist()
            else:
                self.get_logger().info('✓ Emergency stop released')
        
        # Reset Speed (B button) - PRESS
        if self.button_pressed(buttons, self.btn_reset):
            self.speed_multiplier = 1.0
            self.emergency_stopped = False
            self.get_logger().info('✓ Speed reset to 100%')
        
        # Toggle Verbose (Start button) - PRESS
        if self.button_pressed(buttons, self.btn_verbose):
            self.verbose = not self.verbose
            status = 'ON' if self.verbose else 'OFF'
            self.get_logger().info(f'Verbose logging: {status}')
        
        # Update button states for next callback
        self.last_button_states = {i: val for i, val in enumerate(buttons)}

    def button_pressed(self, buttons, button_index):
        """Detect button press (rising edge)"""
        if button_index >= len(buttons):
            return False
        
        current = buttons[button_index] == 1
        previous = self.last_button_states.get(button_index, False)
        
        return current and not previous

    def get_axis_safe(self, axes, index):
        """Safely get axis value with bounds checking"""
        if 0 <= index < len(axes):
            return axes[index]
        return 0.0

    def get_button_safe(self, buttons, index):
        """Safely get button value with bounds checking"""
        if 0 <= index < len(buttons):
            return buttons[index] == 1
        return False

    def apply_deadzone(self, value):
        """Apply deadzone to axis value"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale from deadzone to 1.0
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def publish_cmd_vel(self):
        """Publish velocity commands at fixed rate"""
        self.cmd_vel_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = SwerveGamepadController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocity on shutdown
        if 'controller' in locals():
            zero_twist = Twist()
            controller.cmd_vel_pub.publish(zero_twist)
            controller.get_logger().info('Shutting down - robot stopped')
            controller.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()