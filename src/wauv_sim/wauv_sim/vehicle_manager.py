"""
Filename: vehicle_manager.py
Author: Keiji Toriumi
Description: ROS2 node to wait for vehicle status and manage AUV operation
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

class VehicleManager(Node):

    def __init__(self):
        super().__init__('vehicle_manager')

        # Publish the vehicle ready status
        self.ready_pub = self.create_publisher(
            Bool,
            '/wauv/vehicle_ready',
            10
        )

        # MAVROS service clients to AUV
        self.arm_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.get_logger().info("Waiting for MAVROS services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.get_logger().info("MAVROS services available")

        # track if the AUV is ready
        self.vehicle_ready = False

        # track AUV state
        self.state = State()

        # prevent repeated service spam
        self.mode_request_sent = False
        self.arm_request_sent = False

        # timers to manage retry procedure
        self.mode_timer = self.create_timer(1.0, self.retry_mode)
        self.arm_timer = self.create_timer(1.0, self.retry_arm)


        # main loop
        self.timer = self.create_timer(
            1.0,
            self.manager_loop
        )

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        res = future.result()

        if self.state.mode == "GUIDED":
            self.get_logger().info("Mode confirmed GUIDED")
        else:
            self.get_logger().warn("Mode request sent but not yet active")

    def retry_mode(self):
        if not self.state.connected:
            return
        
        if self.state.mode == "GUIDED":
            return
        
        # try again
        self.set_mode("GUIDED")

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response)

    def arm_response(self, future):
        res = future.result()

        if self.state.armed:
            self.get_logger().info("Vehicle confirmed as armed")
            self.vehicle_ready = True
            self.arm_request_sent = False
        else:
            self.get_logger().warn("Arming failed")
            self.arm_request_sent = False  # allow retry

    def retry_arm(self):
        if not self.state.connected:
            return
        
        # must be guided before armed
        if self.state.mode != "GUIDED":
            return
        
        if not self.state.armed:
            self.vehicle_ready = False
            self.arm_vehicle()
            self.arm_request_sent = True

    def state_callback(self, msg):
        self.state = msg

    def manager_loop(self):
        # wait for FCU connection
        if not self.state.connected:
            self.get_logger().info("Waiting for FCU...")
            return

        self.vehicle_ready = (
            self.state.connected and
            self.state.armed and
            self.state.mode == "GUIDED"
        )

        msg = Bool()
        msg.data = self.vehicle_ready
        self.ready_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = VehicleManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()