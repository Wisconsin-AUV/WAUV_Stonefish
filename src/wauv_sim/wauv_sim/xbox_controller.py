import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
from sensor_msgs.msg import Joy

class XboxController(Node):

    def __init__(self):
        # Initialise the ROS2 node with name 'manual_controller'
        super().__init__('xbox_controller')

        # Publisher sends velocity commands (Twist messages)
        # Topic: MAVROS velocity command topic
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.xbox_sub = self.create_subscription(Joy, '/joy', self.xbox_cb ,10)

        # Timer creates a loop running at 20 Hz (every 0.05 seconds)
        # This repeatedly calls command_loop()
        self.timer = self.create_timer(0.05, self.command_loop)

        # Current velocity state (these get updated by keyboard input)
        self.linear_x = 0.0  # forward/backward
        self.linear_y = 0.0  # left/right
        self.linear_z = 0.0  # up/down
        self.yaw = 0.0

        # Log message to confirm node has started
        self.get_logger().info("XBOX controller started")

    def xbox_cb(self, msg):
        self.linear_y = -1 * msg.axes[0]
        self.linear_x = msg.axes[1]
        self.linear_z = ((1 - msg.axes[5]) / 2) - ((1 - msg.axes[2]) / 2)
        self.yaw = -1 * msg.axes[3]

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

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.yaw

        # Publish command to MAVROS
        self.cmd_pub.publish(cmd)


def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the node
    node = XboxController()

    # Keep node running and processing callbacks
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
