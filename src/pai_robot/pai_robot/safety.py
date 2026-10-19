"""Independent command/range watchdog logic (not certified physical safety)."""
from __future__ import annotations
import math
from .ranges import Ranges

def guard_velocity(v: float, w: float, ranges: Ranges, *, command_age: float,
                   scan_age: float, clock_age: float, max_v: float = 0.12,
                   max_w: float = 0.7) -> tuple[float, float, str]:
    if not all(math.isfinite(x) for x in (v, w, command_age, scan_age, clock_age)):
        return 0.0, 0.0, 'nonfinite'
    if command_age < 0 or command_age > 0.5:
        return 0.0, 0.0, 'command_stale'
    if scan_age < 0 or scan_age > 1.5 or not ranges.valid:
        return 0.0, 0.0, 'scan_stale_or_invalid'
    if clock_age < 0 or clock_age > 0.7:
        return 0.0, 0.0, 'simulation_clock_stalled'
    v = max(-max_v, min(max_v, v))
    w = max(-max_w, min(max_w, w))
    if ranges.nearest < 0.18:
        return 0.0, 0.0, 'object_too_close'
    if (v > 0 and ranges.front < 0.30) or (v < 0 and ranges.rear < 0.30):
        return 0.0, 0.0, 'path_blocked'
    return v, w, 'ok'
