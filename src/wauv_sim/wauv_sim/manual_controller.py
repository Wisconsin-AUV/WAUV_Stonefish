"""
Filename: manual_controller.py
Author: Arhaan Singh
Date: 06/04/2026
Description: ROS2 node to command velocity using keyboard (WASD + QE)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class ManualController(Node):

    def __init__(self):
        # Initialise the ROS2 node with name 'manual_controller'
        super().__init__('manual_controller')

        # Publisher sends velocity commands (Twist messages)
        # Topic: MAVROS velocity command topic
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10  # queue size
        )

        # Timer creates a loop running at 20 Hz (every 0.05 seconds)
        # This repeatedly calls command_loop()
        self.timer = self.create_timer(0.05, self.command_loop)

        # Current velocity state (these get updated by keyboard input)
        self.linear_x = 0.0  # forward/backward
        self.linear_y = 0.0  # left/right
        self.linear_z = 0.0  # up/down
        self.yaw = 0.0
        # Speed multiplier (adjust to control sensitivity)
        self.speed = 1.0
        self.ang_spd = 1.0
        # Start keyboard listener in a separate thread
        # This allows non-blocking input while ROS is spinning
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # Log message to confirm node has started
        self.get_logger().info("WASD manual controller started")

    def on_press(self, key):
        """
        Called whenever a key is pressed.
        Updates velocity values based on key input.
        """
        if key == keyboard.Key.left:
            self.yaw = self.ang_spd
            self.get_logger().info(f"{self.yaw}")
            return
        elif key == keyboard.Key.right:
            self.yaw = -self.ang_spd
            self.get_logger().info(f"{self.yaw}")
            return

        try:
            # Forward
            if key.char == 'w':
                self.linear_x = self.speed

            # Backward
            elif key.char == 's':
                self.linear_x = -self.speed

            # Left
            elif key.char == 'a':
                self.linear_y = self.speed

            # Right
            elif key.char == 'd':
                self.linear_y = -self.speed

            # Up
            elif key.char == 'q':
                self.linear_z = self.speed

            # Down
            elif key.char == 'e':
                self.linear_z = -self.speed

        except AttributeError:
            # Handles special keys (like shift, ctrl) that don’t have .char
            pass

    def on_release(self, key):
        """
        Called when a key is released.
        Stops movement in that direction.
        """
        if key in [keyboard.Key.left, keyboard.Key.right]:
            self.yaw = 0.0
            return

        try:
            # Stop forward/backward motion
            if key.char in ['w', 's']:
                self.linear_x = 0.0

            # Stop left/right motion
            elif key.char in ['a', 'd']:
                self.linear_y = 0.0

            # Stop vertical motion
            elif key.char in ['q', 'e']:
                self.linear_z = 0.0

        except AttributeError:
            pass

    def command_loop(self):
        """
        Runs at 20 Hz.
        Publishes the current velocity as a Twist message.
        """
        cmd = Twist()

        # Assign current velocities to the message
        cmd.linear.x = self.linear_x
        cmd.linear.y = self.linear_y
        cmd.linear.z = self.linear_z

        # Angular velocities are not used (no rotation control yet)
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.yaw

        # Publish command to MAVROS
        self.cmd_pub.publish(cmd)


def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the node
    node = ManualController()

    # Keep node running and processing callbacks
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
