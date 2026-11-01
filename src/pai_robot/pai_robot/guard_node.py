"""Independent 20 Hz command/camera/odometry/clock watchdog."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .safety import guard_velocity
from .motion import OdometrySample, ODOM_MAX_AGE


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        self.v = self.w = 0.0
        self.last_command = self.last_camera = -1e12
        self.last_odom = -1e12
        self.odom_stamp = None
        self.camera_stamp = None
        self.last_sim = None
        self.last_clock_advance = -1e12
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/guard_state', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_command, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady)

    def on_command(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_command = time.monotonic()

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.camera_stamp is None or stamp > self.camera_stamp:
            self.last_camera = time.monotonic()
            self.camera_stamp = stamp

    def tick(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds / 1e9
        if self.last_sim is not None and sim > self.last_sim:
            self.last_clock_advance = now
        if self.last_sim is not None and sim < self.last_sim:
            self.last_command = self.last_camera = -1e12
            self.camera_stamp = None
            self.last_odom, self.odom_stamp = -1e12, None
            self.v = self.w = 0.0
        self.last_sim = sim
        camera_age = now - self.last_camera
        if self.camera_stamp is None or not 0 <= sim - self.camera_stamp <= 0.6:
            camera_age = 1e12
        odom_age = now-self.last_odom
        if self.odom_stamp is None or not 0 <= sim-self.odom_stamp <= ODOM_MAX_AGE:
            odom_age = 1e12
        v, w, reason = guard_velocity(self.v, self.w, command_age=now-self.last_command,
                                     camera_age=camera_age, odom_age=odom_age, clock_age=now-self.last_clock_advance)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)
        status = String()
        status.data = json.dumps({'reason': reason, 'limited': (v, w) != (self.v, self.w),
                                  'sim_time': sim, 'obstacle_detection': False})
        self.status.publish(status)

    def on_odom(self, msg):
        try:
            sample = OdometrySample.from_message(msg)
            if self.odom_stamp is not None and sample.stamp < self.odom_stamp:
                raise ValueError('Odometry clock reset')
            if self.odom_stamp is None or sample.stamp > self.odom_stamp:
                self.last_odom, self.odom_stamp = time.monotonic(), sample.stamp
        except (ValueError, TypeError, AttributeError):
            self.last_odom, self.odom_stamp = -1e12, None


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
