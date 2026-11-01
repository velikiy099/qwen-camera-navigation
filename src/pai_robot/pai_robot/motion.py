"""Wheel-odometry motion control. No scene, map or simulator ground truth input."""
from dataclasses import dataclass
import math

# Robot-side tuning, below the SDF limits (0.15 m/s, 0.8 rad/s, 0.4 m/s²).
MAX_SPEED = 0.12
MAX_TURN_RATE = 0.7
LINEAR_ACCELERATION = 0.3
ANGULAR_ACCELERATION = 1.4
DISTANCE_TOLERANCE = 0.008
ANGLE_TOLERANCE = math.radians(1)
ODOM_MAX_AGE = 0.5


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class OdometrySample:
    stamp: float
    x: float
    y: float
    yaw: float
    linear: float = 0.0
    angular: float = 0.0

    @classmethod
    def from_message(cls, msg):
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        values = (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)
        if not all(math.isfinite(v) for v in values) or stamp < 0:
            raise ValueError('Non-finite or invalid wheel odometry')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0.9 <= norm <= 1.1:
            raise ValueError('Invalid odometry orientation')
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return cls(stamp, p.x, p.y, yaw, twist.linear.x, twist.angular.z)

    @property
    def stationary(self):
        return abs(self.linear) < 0.01 and abs(self.angular) < 0.03


class Motion:
    """Relative straight motion / signed turn, including turns across ±pi."""
    def __init__(self, tool, amount, odom, sim):
        self.tool, self.amount, self.origin = tool, amount, odom
        self.started = self.last_time = sim
        self.last_odom = odom
        self.distance = self.angle = 0.0
        self.v = self.w = 0.0
        self.still_since = None
        self.best_error = abs(amount)
        self.last_progress = sim
        speed = MAX_SPEED if tool == 'move' else MAX_TURN_RATE
        # Allow braking/settling and recovery from lag; this is not a tool limit.
        self.timeout = 5 + 3 * abs(amount) / speed

    def observe(self, odom):
        if odom.stamp > self.last_odom.stamp:
            self.angle += wrap(odom.yaw - self.last_odom.yaw)
            self.last_odom = odom
        dx, dy = odom.x-self.origin.x, odom.y-self.origin.y
        self.distance = dx*math.cos(self.origin.yaw) + dy*math.sin(self.origin.yaw)
        return -dx*math.sin(self.origin.yaw) + dy*math.cos(self.origin.yaw)

    def update(self, sim, odom):
        """Return (linear, angular, outcome). None outcome means still executing."""
        dt = max(0, sim-self.last_time)
        self.last_time = sim
        cross_track = self.observe(odom)
        turn = self.tool == 'turn'
        error = self.amount - (self.angle if turn else self.distance)
        tolerance = ANGLE_TOLERANCE if turn else DISTANCE_TOLERANCE
        heading_error = wrap(self.origin.yaw - odom.yaw)
        at_goal = abs(error) <= tolerance and (turn or abs(heading_error) <= ANGLE_TOLERANCE)
        if at_goal:
            self.v = self.w = 0.0
            if odom.stationary:
                if self.still_since is None:
                    self.still_since = odom.stamp
                if odom.stamp - self.still_since >= 0.2:
                    return 0.0, 0.0, 'completed'
            else:
                self.still_since = None
        else:
            self.still_since = None
        if self.best_error - abs(error) >= tolerance / 4:
            self.best_error, self.last_progress = abs(error), sim
        if sim-self.started > self.timeout or (not at_goal and sim-self.last_progress > 3):
            self.v = self.w = 0.0
            return 0.0, 0.0, 'motion_failed'
        if at_goal:
            return 0.0, 0.0, None
        if turn:
            desired_v, desired_w = 0.0, clamp(1.5*error, MAX_TURN_RATE)
        else:
            desired_v = clamp(error, MAX_SPEED) * max(0.0, math.cos(heading_error))
            direction = 1 if self.amount >= 0 else -1
            desired_w = clamp(2*heading_error - direction*2*cross_track, MAX_TURN_RATE)
        self.v += clamp(desired_v-self.v, LINEAR_ACCELERATION*dt)
        self.w += clamp(desired_w-self.w, ANGULAR_ACCELERATION*dt)
        return self.v, self.w, None
