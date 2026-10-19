"""Conservative conversion of a 360-degree LaserScan into range sectors."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

@dataclass(frozen=True)
class Ranges:
    front: float
    left: float
    right: float
    rear: float
    nearest: float
    valid: bool

    @classmethod
    def invalid(cls) -> 'Ranges':
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, False)

def summarize_scan(values: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> Ranges:
    if (not values or not all(math.isfinite(x) for x in (angle_min, angle_increment, range_min, range_max))
            or angle_increment <= 0 or not 0 <= range_min < range_max):
        return Ranges.invalid()
    buckets = {key: [] for key in ('front', 'left', 'right', 'rear')}
    invalid = {key: 0 for key in buckets}
    all_valid = []
    for i, value in enumerate(values):
        angle = math.atan2(math.sin(angle_min + i * angle_increment), math.cos(angle_min + i * angle_increment))
        deg = math.degrees(angle)
        key = ('front' if abs(deg) <= 20 else 'left' if 20 < deg <= 100
               else 'right' if -100 <= deg < -20 else 'rear')
        # Positive infinity is a legitimate "no return within range" value.
        if value == math.inf:
            value = range_max
        elif value == -math.inf or (math.isfinite(value) and value < range_min):
            # Saturated/too-close beam: fail safe rather than drop it.
            value = 0.0
        elif not math.isfinite(value) or value > range_max:
            invalid[key] += 1
            continue
        buckets[key].append(float(value))
        all_valid.append(float(value))
    # Do not call an unobserved direction free space. This world uses a full scan.
    valid = all(buckets[k] and invalid[k] <= len(buckets[k]) for k in buckets)
    if not valid:
        return Ranges.invalid()
    return Ranges(*(min(buckets[k]) for k in ('front', 'left', 'right', 'rear')),
                  min(all_valid), True)
