#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import socket, struct, json
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ArduSimPatch(Node):
    def __init__(self):
        super().__init__('ardusim_patch', namespace='bluerov2')

        # State
        self.imu = None
        self.odom = None
        self._t0 = None

        # Subscribers
        self.create_subscription(Imu, 'imu', self._imu_cb, 1)
        self.create_subscription(Odometry, 'odometry', self._odom_cb, 1)

        # Publisher
        self.pub_pwm = self.create_publisher(Float64MultiArray, 'setpoint/pwm', 1)

        # UDP socket — ArduSub SITL JSON interface
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 9002))
        self.sock.settimeout(0.01)  # 10ms — non-blocking enough

        # Timer at 50Hz
        self.create_timer(1/100, self._loop)

    def _imu_cb(self, msg):
        self.imu = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _loop(self):
        # Wait for sensors
        if self.imu is None or self.odom is None:
            return

        # Try to receive PWM packet from ArduSub
        try:
            data, address = self.sock.recvfrom(100)
        except Exception:
            return  # SITL not ready yet, just skip this tick

        # Parse PWM packet
        fmt = 'HHI16H'
        if len(data) != struct.calcsize(fmt):
            return
        decoded = struct.unpack(fmt, data)
        if decoded[0] != 18458:  # magic
            return

        pwm = decoded[3:]
        pwm_thrusters = pwm[0:8]
        # Normalize: 1500 = zero, 400 = full scale
        setpoints = [(x - 1500) / 400.0 for x in pwm_thrusters]
        self.pub_pwm.publish(Float64MultiArray(data=setpoints))

        # --- Build JSON to send back ---

        # Monotonic timestamp in MILLISECONDS (ArduSub JSON expects ms)
        now_ns = self.get_clock().now().nanoseconds
        if self._t0 is None:
            self._t0 = now_ns
        timestamp_s = (now_ns - self._t0) / 1e9

        # IMU — already FRD, pass through
        ax = self.imu.linear_acceleration.x
        ay = self.imu.linear_acceleration.y
        az = self.imu.linear_acceleration.z
        gx = self.imu.angular_velocity.x
        gy = self.imu.angular_velocity.y
        gz = self.imu.angular_velocity.z

        # Position — already world NED, pass through
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        pz = self.odom.pose.pose.position.z

        # Attitude — quaternion already NED body, euler is already FRD
        q = self.odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        att = [roll, pitch, yaw]

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

        vx_b = self.odom.twist.twist.linear.x
        vy_b = self.odom.twist.twist.linear.y
        vz_b = self.odom.twist.twist.linear.z

        vx = R00*vx_b + R01*vy_b + R02*vz_b
        vy = R10*vx_b + R11*vy_b + R12*vz_b
        vz = R20*vx_b + R21*vy_b + R22*vz_b

        payload = {
            'timestamp': timestamp_s,
            'imu': {
                'gyro': [gx, gy, gz],
                'accel_body': [ax, ay, az]
            },
            'position': [px, py, pz],
            'attitude': att,
            'velocity': [vx, vy, vz]
        }

        msg = '\n' + json.dumps(payload, separators=(',', ':')) + '\n'
        if not hasattr(self, '_print_tick'):
            self._print_tick = 0
        self._print_tick += 1

        if self._print_tick % 50 == 0:
            print(f"SENDING: {msg.strip()}")
        self.sock.sendto(msg.encode('ascii'), address)



def main(args=None):
    rclpy.init(args=args)
    node = ArduSimPatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
