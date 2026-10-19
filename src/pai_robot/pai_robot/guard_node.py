"""Independent ROS node: stale commands/sensors/clock produce zero Twist."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .ranges import Ranges, summarize_scan
from .safety import guard_velocity

class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        self.v = self.w = 0.0
        self.last_command = self.last_scan = 0.0
        self.last_sim = None
        self.last_clock_advance = 0.0
        self.ranges = Ranges.invalid()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady)
    def on_command(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_command = time.monotonic()
    def on_scan(self, msg):
        self.ranges = summarize_scan(list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        self.last_scan = time.monotonic()
    def tick(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds
        if self.last_sim is not None and sim > self.last_sim:
            self.last_clock_advance = now
        if self.last_sim is not None and sim < self.last_sim:
            self.last_command = 0.0
        self.last_sim = sim
        v, w, _ = guard_velocity(self.v, self.w, self.ranges, command_age=now - self.last_command,
                                 scan_age=now - self.last_scan, clock_age=now - self.last_clock_advance)
        msg = Twist(); msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)

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
