"""Pure finite-state visual controller. Outputs body Twist, not wheel commands.

AI verifies an image; it does NOT generate code or wheel controls in this version.
"""
from __future__ import annotations
from dataclasses import dataclass
from .vision_tracking import ColorBlob
from .ranges import Ranges

@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0
    state: str = 'IDLE'
    reason: str = ''
    request_vision: bool = False

class VisualSearchBrain:
    def __init__(self, target: str, *, mission_timeout: float = 240.0):
        if target not in ('red', 'blue'):
            raise ValueError('Target must be red or blue')
        self.target = target
        self.state = 'SEARCH'
        self.reason = 'Waiting for target candidate'
        self.elapsed = 0.0
        self.active_time = 0.0
        self.mission_timeout = mission_timeout
        self.visible_frames = 0
        self.lost_frames = 0
        self.approved = False
        self.avoid_count = 0
        self.turn_sign = 1.0
        self.cooldown = 0.0

    def transition(self, state: str, reason: str):
        self.state, self.reason, self.elapsed = state, reason, 0.0
        self.visible_frames = 0

    def fail(self, reason: str):
        self.approved = False
        self.transition('ERROR', reason)

    def record_vision(self, colors: set[str]):
        if self.state != 'VERIFY':
            raise RuntimeError('Vision result is only accepted in VERIFY')
        self.approved = self.target in colors
        if self.approved:
            self.transition('APPROACH', 'VLM confirmed target color; pixel tracking controls motion')
        else:
            self.cooldown = 1.5
            self.transition('SEARCH', 'VLM did not confirm target')

    def step(self, blob: ColorBlob | None, candidate: bool, ranges: Ranges,
             dt: float, *, new_image: bool = True) -> Command:
        if dt < 0:
            raise ValueError('dt must be non-negative')
        if self.state in ('ERROR', 'DONE'):
            return Command(state=self.state, reason=self.reason)
        if not ranges.valid:
            return Command(state=self.state, reason='No valid full lidar scan; stopped')
        self.elapsed += dt
        # Slow VLM inference does not exhaust the movement budget.
        if self.state not in ('VERIFY', 'SETTLE'):
            self.active_time += dt
        self.cooldown = max(0.0, self.cooldown - dt)
        if self.active_time > self.mission_timeout:
            self.fail('Movement timeout')
            return Command(state=self.state, reason=self.reason)
        if self.state == 'VERIFY':
            return Command(state='VERIFY', reason='Stopped during VLM inference', request_vision=True)
        if self.state == 'SETTLE':
            if self.elapsed >= 0.5 and new_image:
                self.transition('VERIFY', 'Robot has settled; verify a fresh camera frame')
                return Command(state='VERIFY', reason=self.reason, request_vision=True)
            return Command(state='SETTLE', reason='Stopping before taking VLM snapshot')
        if self.state == 'SEARCH':
            if new_image:
                self.visible_frames = self.visible_frames + 1 if candidate else 0
            if self.visible_frames >= 3 and self.cooldown <= 0.0:
                self.transition('SETTLE', 'Colored candidate in three distinct images')
                return Command(state='SETTLE', reason=self.reason)
            return Command(0.0, 0.35, 'SEARCH', 'Rotate and observe camera')
        if self.state == 'AVOID':
            if self.elapsed < 0.5:
                return Command(-0.04 if ranges.rear > 0.40 else 0.0, 0.0, 'AVOID', 'Short guarded retreat')
            if self.elapsed < 2.5:
                return Command(0.0, self.turn_sign * 0.5, 'AVOID', 'Turn toward larger free sector')
            if self.elapsed < 3.7 and ranges.front > 0.5:
                return Command(0.07, 0.0, 'AVOID', 'Short guarded bypass')
            self.approved = False
            self.cooldown = 0.6
            self.transition('SEARCH', 'Bypass attempted; observe and verify again')
            return Command(state='SEARCH', reason=self.reason)
        if self.state != 'APPROACH' or not self.approved:
            self.fail('Unexpected or unverified state')
            return Command(state='ERROR', reason=self.reason)
        if blob is None:
            if new_image:
                self.lost_frames += 1
            if self.lost_frames >= 5:
                self.approved = False
                self.lost_frames = 0
                self.transition('SEARCH', 'Target lost; another VLM confirmation will be required')
            return Command(state=self.state, reason='No current target pixels; stopped')
        self.lost_frames = 0
        centered = abs(blob.center_x_norm) < 0.24
        # This is a heuristic range association, not object-ground-truth distance.
        if centered and blob.width_fraction > 0.24 and blob.pixel_fraction > 0.035 and ranges.front < 0.40:
            self.transition('DONE', 'Visible centered target + front lidar proximity')
            return Command(state='DONE', reason=self.reason)
        # A centered, sufficiently large verified target is expected to be near.
        # Do not treat that target itself as an obstruction before the stop threshold.
        target_in_front = centered and blob.width_fraction > 0.22 and blob.pixel_fraction > 0.025
        if ranges.front < 0.50 and not target_in_front:
            self.avoid_count += 1
            if self.avoid_count > 5:
                self.fail('Repeated obstruction; maximum avoidance attempts reached')
                return Command(state='ERROR', reason=self.reason)
            self.turn_sign = 1.0 if ranges.left >= ranges.right else -1.0
            self.approved = False
            self.transition('AVOID', 'Front obstruction before arrival criteria')
            return Command(state='AVOID', reason=self.reason)
        # ROS REP-103: positive yaw turns left; image positive x points right.
        error = blob.center_x_norm
        yaw = max(-0.65, min(0.65, -1.1 * error))
        speed = 0.0 if abs(error) > 0.50 else (0.11 if abs(error) < 0.18 else 0.07)
        return Command(speed, yaw, 'APPROACH', 'Image-based proportional steering')
