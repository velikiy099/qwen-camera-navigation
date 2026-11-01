"""Command/camera/odometry/clock watchdog. No obstacle sensing."""
import math
from .motion import MAX_SPEED, MAX_TURN_RATE, ODOM_MAX_AGE


def guard_velocity(v, w, *, command_age, camera_age, odom_age, clock_age):
    if not all(math.isfinite(x) for x in (v, w, command_age, camera_age, odom_age, clock_age)):
        return 0.0, 0.0, 'nonfinite'
    if not 0 <= command_age <= 0.5:
        return 0.0, 0.0, 'command_stale'
    if not 0 <= camera_age <= 1.5:
        return 0.0, 0.0, 'camera_stale'
    if not 0 <= odom_age <= ODOM_MAX_AGE:
        return 0.0, 0.0, 'odometry_stale'
    if not 0 <= clock_age <= 0.7:
        return 0.0, 0.0, 'simulation_clock_stalled'
    limited_v = max(-MAX_SPEED, min(MAX_SPEED, v))
    limited_w = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, w))
    return limited_v, limited_w, 'velocity_limit' if (v, w) != (limited_v, limited_w) else 'ok'
