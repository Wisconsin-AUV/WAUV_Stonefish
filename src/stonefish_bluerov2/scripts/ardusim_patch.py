#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import socket, struct, json
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ArduSimPatch(Node):
    def __init__(self):
        super().__init__('ardusim_patch', namespace='bluerov2')
        
        # Shared State & Thread Safety
        self.lock = threading.Lock()
        self.imu = None
        self.odom = None
        
        # Virtual Simulation Time
        self._t_sim = 0.0

        # Subscribers
        self.create_subscription(Imu, 'imu', self._imu_cb, 1)
        self.create_subscription(Odometry, 'odometry', self._odom_cb, 1)
        
        # Publisher
        self.pub_pwm = self.create_publisher(Float64MultiArray, 'setpoint/pwm', 1)
        
        # UDP socket — ArduSub SITL JSON interface
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 9002))
        # Use a small timeout so the thread can occasionally check rclpy.ok() to shutdown cleanly
        self.sock.settimeout(0.5)  
        
        # Start background thread for UDP communication
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

    def _imu_cb(self, msg):
        with self.lock:
            self.imu = msg

    def _odom_cb(self, msg):
        with self.lock:
            self.odom = msg

    def _udp_loop(self):
        fmt = 'HHI16H'
        expected_size = struct.calcsize(fmt)
        print_tick = 0

        # Run as long as the ROS2 node is alive
        while rclpy.ok():
            try:
                # This will block up to 0.5 seconds, offloading the ROS2 main thread entirely
                data, address = self.sock.recvfrom(100)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Socket error: {e}")
                continue

            if len(data) != expected_size:
                continue

            decoded = struct.unpack(fmt, data)
            if decoded[0] != 18458:  # magic number check
                continue

            # Extract frame rate to determine our virtual time step
            frame_rate = decoded[1]
            dt = (1.0 / frame_rate) if frame_rate > 0 else 0.01

            # Extract PWMs
            pwm_thrusters = decoded[3:11]
            
            # Normalize: 1500 = zero, 400 = full scale
            setpoints = [(x - 1500) / 400.0 for x in pwm_thrusters]
            self.pub_pwm.publish(Float64MultiArray(data=setpoints))

            # Safely grab the latest sensor data
            with self.lock:
                if self.imu is None or self.odom is None:
                    continue
                # Copy references locally to minimize lock time
                imu_msg = self.imu
                odom_msg = self.odom

            # Increment virtual clock by the exact dt expected by ArduSub
            self._t_sim += dt

            # --- Build JSON to send back ---
            ax = imu_msg.linear_acceleration.x
            ay = imu_msg.linear_acceleration.y
            az = imu_msg.linear_acceleration.z
            gx = imu_msg.angular_velocity.x
            gy = imu_msg.angular_velocity.y
            gz = imu_msg.angular_velocity.z

            px = odom_msg.pose.pose.position.x
            py = odom_msg.pose.pose.position.y
            pz = odom_msg.pose.pose.position.z

            q = odom_msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Velocity — rotate body NED to world NED
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            R00 = 1 - 2*(qy**2 + qz**2)
            R01 = 2*(qx*qy - qz*qw)
            R02 = 2*(qx*qz + qy*qw)
            R10 = 2*(qx*qy + qz*qw)
            R11 = 1 - 2*(qx**2 + qz**2)
            R12 = 2*(qy*qz - qx*qw)
            R20 = 2*(qx*qz - qy*qw)
            R21 = 2*(qy*qz + qx*qw)
            R22 = 1 - 2*(qx**2 + qy**2)

            vx_b = odom_msg.twist.twist.linear.x
            vy_b = odom_msg.twist.twist.linear.y
            vz_b = odom_msg.twist.twist.linear.z

            vx = R00*vx_b + R01*vy_b + R02*vz_b
            vy = R10*vx_b + R11*vy_b + R12*vz_b
            vz = R20*vx_b + R21*vy_b + R22*vz_b

            payload = {
                'timestamp': self._t_sim,
                'imu': {
                    'gyro': [gx, gy, gz],
                    'accel_body': [ax, ay, az]
                },
                'position': [px, py, pz],
                'attitude': [roll, pitch, yaw],
                'velocity': [vx, vy, vz]
            }

            msg = '\n' + json.dumps(payload, separators=(',', ':')) + '\n'
            
            print_tick += 1
            if print_tick % 50 == 0:
                print(f"SENDING (dt={dt:.3f}): {msg.strip()}")
                
            try:
                self.sock.sendto(msg.encode('ascii'), address)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ArduSimPatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()