"""Camera-based tool decisions; wheel odometry stays inside robot-side control."""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
import uuid
from .motion import Motion, ODOM_MAX_AGE
from .navigation import COLORS, SHAPES, ROUTE_DIRECTIONS, ROUTE_STATES, DEFAULT_CLEARANCE_M, GEOMETRY, Goal, SearchProgress, parse_goal, finite_number

SETTLE_SECONDS = 0.5
ARRIVAL_RECHECK_LIMIT = 3
TOOLS = ('move', 'turn', 'observe', 'finish', 'stop')

_BASE_SCHEMA = {
    'type': 'object',
    'properties': {
        'observation': {'type': 'string', 'maxLength': 500},
        'target_visible': {'type': 'boolean'},
        'target_reached': {'type': 'boolean'},
        'memory': {'type': 'string', 'maxLength': 800},
        'candidate_color': {'type': 'string', 'enum': list(COLORS)},
        'candidate_shape': {'type': 'string', 'enum': list(SHAPES)},
        'ground_contact_px': {'type': 'array', 'items': {'type': 'number'}, 'maxItems': 2},
        'routes': {'type': 'object', 'properties': {d: {'type': 'string', 'enum': list(ROUTE_STATES)} for d in ROUTE_DIRECTIONS},
                   'required': list(ROUTE_DIRECTIONS), 'additionalProperties': False},
        'next_subgoal': {'type': 'string', 'maxLength': 300},
        'tool': {'type': 'string', 'enum': list(TOOLS)},
        'distance_m': {'type': 'number'},
        'direction': {'type': 'string', 'enum': ['left', 'right', 'none']},
        'angle_deg': {'type': 'number', 'minimum': 0},
    },
    'additionalProperties': False,
}
TOOL_FIELDS = tuple(_BASE_SCHEMA['properties'])
_BASE_SCHEMA['required'] = list(TOOL_FIELDS)


def _tool_schema(tool):
    # Separate grammar branches prevent the model filling unused motion fields.
    properties = {key: dict(value) for key, value in _BASE_SCHEMA['properties'].items()}
    properties['tool'] = {'const': tool}
    properties['direction'] = ({'type': 'string', 'enum': ['left', 'right']}
                               if tool == 'turn' else {'const': 'none'})
    active = {'move': ('distance_m',), 'turn': ('angle_deg',)}.get(tool, ())
    for key in ('distance_m', 'angle_deg'):
        if key not in active:
            properties[key] = {'const': 0}
    return {**_BASE_SCHEMA, 'properties': properties}


ACTION_SCHEMA = {'anyOf': [_tool_schema(tool) for tool in TOOLS]}


@dataclass(frozen=True)
class ToolCall:
    tool: str
    distance_m: float
    direction: str
    angle_deg: float
    observation: str
    memory: str
    target_visible: bool
    target_reached: bool
    candidate_color: str
    candidate_shape: str
    ground_contact_px: list
    routes: dict
    next_subgoal: str

    @property
    def signed_angle_deg(self):
        """Convert the validated model direction to the robot's yaw convention."""
        return -self.angle_deg if self.direction == 'right' else self.angle_deg


def parse_action(raw: str) -> ToolCall:
    value = json.loads(raw)
    if not isinstance(value, dict) or set(value) != set(TOOL_FIELDS):
        raise ValueError('Exactly one complete tool call is required')
    if not isinstance(value['tool'], str) or value['tool'] not in TOOLS:
        raise ValueError('Unknown tool')
    for key in ('distance_m', 'angle_deg'):
        x = value[key]
        try:
            finite = math.isfinite(x)
        except (TypeError, OverflowError):
            finite = False
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not finite:
            raise ValueError(f'{key} must be a finite number')
    for key, limit in (('observation', 500), ('memory', 800), ('next_subgoal', 300)):
        if not isinstance(value[key], str) or len(value[key]) > limit:
            raise ValueError(f'Invalid {key}')
    if not value['observation'].strip():
        raise ValueError('Describe the current camera observation')
    if any(type(value[k]) is not bool for k in ('target_visible', 'target_reached')):
        raise ValueError('Target flags must be booleans')
    if value['candidate_color'] not in COLORS or value['candidate_shape'] not in SHAPES:
        raise ValueError('Describe the observed candidate color and shape without changing the goal')
    point = value['ground_contact_px']
    if not isinstance(point, list) or len(point) not in (0, 2) or any(not finite_number(v) for v in point):
        raise ValueError('ground_contact_px must be [] or [u, v] in original image pixels')
    routes = value['routes']
    if (not isinstance(routes, dict) or set(routes) != set(ROUTE_DIRECTIONS)
            or any(v not in ROUTE_STATES for v in routes.values())):
        raise ValueError('routes must describe left/forward/right/backward as clear, blocked or unknown')
    a = ToolCall(**value)
    directions = ('left', 'right') if a.tool == 'turn' else ('none',)
    if not isinstance(a.direction, str) or a.direction not in directions:
        raise ValueError('turn requires direction=left or right; other tools require direction=none')
    if a.angle_deg < 0:
        raise ValueError('angle_deg must be nonnegative; choose direction=left or right for turns')
    if a.tool == 'move':
        if a.angle_deg != 0:
            raise ValueError('move requires angle_deg=0; turn first, then move')
    elif a.tool == 'turn':
        if a.distance_m != 0:
            raise ValueError('turn requires distance_m=0; move separately')
    elif any(value[k] != 0 for k in ('distance_m', 'angle_deg')):
        raise ValueError('Stationary tools require zero motion fields')
    return a


def parse_mission(raw: str) -> str:
    value = json.loads(raw)
    if not isinstance(value, dict) or not {'instruction'} <= set(value) <= {'instruction', 'mission_id'}:
        raise ValueError('Mission must contain instruction and optionally mission_id')
    if 'mission_id' in value:
        mission_id = value['mission_id']
        if not isinstance(mission_id, str) or len(mission_id) != 32:
            raise ValueError('mission_id must be a UUID hex string')
        try:
            uuid.UUID(hex=mission_id)
        except ValueError as exc:
            raise ValueError('mission_id must be a UUID hex string') from exc
    instruction = value['instruction']
    if not isinstance(instruction, str) or not 1 <= len(instruction.strip()) <= 2000:
        raise ValueError('Instruction must contain 1..2000 characters')
    return instruction.strip()


class CameraAgent:
    """One tool at a time; the adapter provides images and asynchronous inference."""
    def __init__(self, instruction: str, sim: float, *, max_actions=80, max_sim_seconds=900.0):
        self.instruction = instruction
        self.max_actions = max_actions
        self.max_sim_seconds = max_sim_seconds
        self.start_sim = sim
        self.state = 'SETTLING'
        self.reason = 'Waiting for a fresh stationary camera frame'
        self.ready_at = sim + SETTLE_SECONDS
        self.last_sim = sim
        self.action = None
        self.motion = None
        self.odom = None
        self.action_started = sim
        self.actions = 0
        self.inference_calls = 0
        self.memory = ''
        self.history = []
        self.recent_results = []
        self._goal = None
        self.search = SearchProgress()
        self.assessment = {}
        self.previous_near_frame = None
        self.arrival_hold = None
        self.completion_source = None

    @property
    def goal(self):
        return self._goal

    def lock_goal(self, goal):
        if self._goal is not None:
            raise ValueError('The mission goal is immutable; send a new mission to change it')
        self._goal = parse_goal(json.dumps(asdict(goal), allow_nan=False))

    def observe_frame(self, png):
        self.search.observe_frame(png)

    @property
    def terminal(self):
        return self.state in ('DONE', 'STOPPED', 'HALTED', 'ERROR')

    def stop(self, state: str, reason: str, sim: float):
        if self.action is not None:
            self._result('interrupted', sim, reason)
        self.action = None
        self.motion = None
        self.state, self.reason = state, reason

    def _result(self, status, sim, reason='', proposed_action=None):
        a = self.action
        elapsed = max(0.0, sim - self.action_started)
        if self.motion and self.odom:
            self.motion.observe(self.odom)
        result = {
            'step': self.inference_calls, 'tool_call': asdict(a), 'status': status,
            'started_sim_s': self.action_started, 'ended_sim_s': sim,
            'elapsed_sim_s': round(elapsed, 4),
            'odometry_distance_m': self.motion.distance if self.motion else None,
            'odometry_angle_deg': math.degrees(self.motion.angle) if self.motion else None,
            'motion_feedback': 'wheel_odometry' if self.motion else 'none',
            'note': 'Wheel odometry is an estimate; wheel slip and collisions are not measured.',
            'reason': reason,
        }
        if proposed_action is not None:
            result['proposed_tool_call'] = asdict(proposed_action)
        self._record(result)
        if status == 'completed':
            self.search.motion_completed(a)
        self.action = self.motion = None

    def _record(self, result):
        self.history.append(result)
        self.history = self.history[-12:]
        self.recent_results.append(result)

    def advance(self, sim: float, odom=None):
        if not math.isfinite(sim) or sim < self.last_sim:
            self.stop('ERROR', 'Simulation clock moved backwards or became invalid', self.last_sim)
            return 0.0, 0.0
        dt = sim - self.last_sim
        self.last_sim = sim
        if odom is not None:
            self.odom = odom
        if self.terminal:
            return 0.0, 0.0
        if sim - self.start_sim > self.max_sim_seconds:
            self.stop('ERROR', 'Mission simulation-time budget exceeded', sim)
            return 0.0, 0.0
        if self.motion is not None:
            if dt > 0.25:
                self.stop('ERROR', 'Control timing gap during motion', sim)
                return 0.0, 0.0
            if self.odom is None or not 0 <= sim-self.odom.stamp <= ODOM_MAX_AGE:
                self.stop('ERROR', 'Wheel odometry is stale or missing', sim)
                return 0.0, 0.0
            v, w, outcome = self.motion.update(sim, self.odom)
            if outcome:
                reason = ('Robot-side controller reached the requested relative motion using wheel odometry'
                          if outcome == 'completed' else 'Robot stopped: wheel odometry did not converge; inspect the new image and choose another action')
                self._result(outcome, sim, reason)
                self.state = 'SETTLING'
                self.reason = reason
                self.ready_at = sim + SETTLE_SECONDS
                return 0.0, 0.0
            return v, w
        return 0.0, 0.0

    def ready(self, sim, image_time):
        return (self.state == 'SETTLING' and sim >= self.ready_at and image_time >= self.ready_at
                and self.odom is not None and self.odom.stationary
                and 0 <= sim-self.odom.stamp <= ODOM_MAX_AGE)

    def begin_inference(self):
        if self.state != 'SETTLING':
            raise RuntimeError('Inference requires a stationary observation')
        call_cost = 1  # Goal once; each image produces observation + action together.
        if self.inference_calls + call_cost > self.max_actions:
            self.state, self.reason = 'ERROR', 'Tool-call budget exceeded'
            return False
        self.inference_calls += call_cost
        self.state, self.reason = 'THINKING', ('Stopped while Qwen interprets the mission once'
                                             if self.goal is None else 'Stopped while Qwen chooses the next tool')
        return True

    def reject(self, reason, sim):
        """A malformed tool is feedback for the next inference, not actuation."""
        self._record({'step': self.inference_calls, 'tool_call': None, 'status': 'rejected', 'reason': reason})
        self.state, self.reason = 'SETTLING', 'Tool rejected; stopped while Qwen corrects the arguments'
        self.ready_at = sim + SETTLE_SECONDS

    def _hold_for_arrival(self, action, estimate, near, confirmed, sim):
        """Keep the last near estimate only as a motion hold, never as new evidence."""
        frame = self.search.frame_id
        if near:
            self.arrival_hold = {'last_near_estimate': estimate, 'last_near_frame': frame,
                                 'last_checked_frame': frame, 'unconfirmed_frames': 0}
        elif estimate is not None:
            # A fresh matching candidate beyond the arrival distance permits approach again.
            self.arrival_hold = None
        elif self.arrival_hold and frame > self.arrival_hold['last_checked_frame']:
            self.arrival_hold['unconfirmed_frames'] += 1
            self.arrival_hold['last_checked_frame'] = frame
        self.assessment['arrival_confirmation_pending'] = bool(self.arrival_hold and not confirmed)
        if not self.arrival_hold or action.tool == 'stop':
            return False
        if confirmed:
            tool, state, outcome = 'finish', 'DONE', 'arrival_confirmed'
            reason = 'Camera-based arrival confirmed in two fresh observations while stopped; no independent distance/contact verification'
            self.completion_source = 'camera_arrival_policy'
        elif self.arrival_hold['unconfirmed_frames'] >= ARRIVAL_RECHECK_LIMIT:
            tool, state, outcome = 'stop', 'STOPPED', 'arrival_unconfirmed'
            reason = 'Arrival recheck failed in three fresh observations; staying stopped near the last estimated target position. Arrival is unconfirmed'
            self.assessment['arrival_confirmation_pending'] = False
        else:
            tool, state, outcome = 'observe', 'SETTLING', 'arrival_recheck'
            reason = 'Within arrival distance in a previous or current image; hold position and confirm the target ground boundary in a fresh image'
        # Preserve the model proposal for auditing; tool_call describes what actually ran.
        self.action = replace(action, tool=tool, distance_m=0, direction='none', angle_deg=0,
                              target_reached=confirmed)
        self.action_started = sim
        self.actions += 1
        self.memory = action.memory
        self._result(outcome, sim, reason, proposed_action=action)
        self.state, self.reason = state, reason
        self.ready_at = sim + SETTLE_SECONDS
        return True

    def accept(self, action: ToolCall, sim):
        if self.state != 'THINKING':
            raise RuntimeError('Unexpected model response')
        if isinstance(action, Goal):
            self.lock_goal(action)
            self.state = 'STOPPED' if action.kind == 'unsupported' else 'SETTLING'
            self.reason = ('Unsupported or ambiguous mission: ' if action.kind == 'unsupported' else 'Fixed mission goal: ') + action.description
            self.ready_at = sim + SETTLE_SECONDS
            return
        if self.goal is None:
            self.reject('Interpret and lock the mission goal before choosing motion', sim)
            return
        # Validate again at the actuation boundary, even for an injected/mock client.
        action = parse_action(json.dumps(asdict(action), allow_nan=False))
        matched = (self.goal.kind == 'approach' and action.candidate_color not in ('unknown', 'none')
                   and action.candidate_shape not in ('unknown', 'none')
                   and self.goal.color in ('unspecified', action.candidate_color)
                   and self.goal.shape in ('unspecified', action.candidate_shape))
        estimate = (GEOMETRY.clearance(action.ground_contact_px, self.search.image_size, self.search.last_png)
                    if matched and self.search.last_png is not None else None)
        near = bool(estimate and estimate['clearance_m'] <= self.goal.max_clearance_m)
        confirmed = near and self.previous_near_frame is not None and self.previous_near_frame < self.search.frame_id
        self.previous_near_frame = self.search.frame_id if near else None
        self.assessment = {'candidate_matches_goal': matched, 'distance_estimate': estimate,
                           'within_arrival_distance': near, 'confirmed_in_two_frames': bool(confirmed),
                           'frame': self.search.frame_id, 'arrival_reference': 'vehicle front to target near surface',
                           'independently_verified': False}
        self.search.record(action, matched)
        if self.goal.kind == 'approach':
            if self._hold_for_arrival(action, estimate, near, confirmed, sim):
                return
            if (action.target_visible or action.target_reached or action.tool == 'finish') and not matched:
                self.reject(f'Candidate {action.candidate_color} {action.candidate_shape} does not match fixed goal '
                            f'{self.goal.color} {self.goal.shape}. Do not substitute another target. Continue searching.', sim)
                return
            if action.tool == 'finish' and not confirmed:
                self.reject(f'Arrival not confirmed: require matching target and visible ground boundary within '
                            f'{self.goal.max_clearance_m:g} m of vehicle front in two fresh observations. '
                            f'Current image-based estimate: {estimate}. Discovery alone is not arrival.', sim)
                return
        elif action.tool == 'finish' and self.search.frame_id < 2:
            self.reject('Inspect the environment before reporting inspection complete', sim)
            return
        if action.tool == 'move' and action.distance_m != 0:
            direction = 'forward' if action.distance_m > 0 else 'backward'
            if action.routes[direction] != 'clear':
                self.reject(f'Move contradicts current observation: {direction} route is '
                            f'{action.routes[direction]}. Inspect a visible route before translating. '
                            'A front camera does not confirm the space behind the robot.', sim)
                return
        self.actions += 1
        self.memory = action.memory
        self.action, self.action_started = action, sim
        if action.tool in ('move', 'turn'):
            if self.odom is None or not 0 <= sim-self.odom.stamp <= ODOM_MAX_AGE:
                raise RuntimeError('Fresh wheel odometry is required before motion')
            amount = action.distance_m if action.tool == 'move' else math.radians(action.signed_angle_deg)
            self.previous_near_frame = None
            self.motion = Motion(action.tool, amount, self.odom, sim)
            self.state, self.reason = 'EXECUTING', action.observation
        else:
            if action.tool == 'finish':
                self.completion_source = 'model'
            self._result('observed' if action.tool == 'observe' else 'model_reported', sim)
            self.ready_at = sim + SETTLE_SECONDS
            self.state = {'observe': 'SETTLING', 'finish': 'DONE', 'stop': 'STOPPED'}[action.tool]
            self.reason = ('Camera-based goal checks passed; no independent arrival/contact verification'
                           if action.tool == 'finish' else action.observation)

    def context(self):
        # Keep old model descriptions from drowning out the current image.
        results = []
        for result in self.history:
            motion = {k: v for k, v in (result['tool_call'] or {}).items()
                      if k in ('tool', 'distance_m', 'direction', 'angle_deg')}
            results.append({'step': result['step'], **motion, 'status': result['status'],
                            'reason': result['reason']})
        return {'instruction': self.instruction, 'step': max(1, self.inference_calls),
                'task': 'interpret_mission' if self.goal is None else 'navigate',
                'fixed_goal': asdict(self.goal) if self.goal else None,
                'arrival_policy': {'default_max_clearance_m': DEFAULT_CLEARANCE_M,
                    'reference': 'vehicle front to target near surface',
                    'requires': 'matching color/shape and camera-based distance within goal in two fresh observations while stopped',
                    'on_first_near_estimate': 'hold position; do not move or turn; re-observe',
                    'on_confirmation': 'code finishes automatically, regardless of proposed action',
                    'on_lost_boundary': 'hold position; missing estimates do not count as arrival evidence',
                    'max_unconfirmed_recheck_frames': ARRIVAL_RECHECK_LIMIT},
                'arrival_hold': self.arrival_hold,
                'camera_and_robot': GEOMETRY.context(),
                'last_candidate_assessment': self.assessment,
                'search_progress': self.search.context(),
                'remaining_calls': self.max_actions - self.inference_calls,
                'memory_from_previous_model_call': self.memory,
                'recent_tool_results': results,
                'sensors': 'single forward RGB camera only',
                'motion_feedback': 'robot-side wheel-odometry execution status only; no poses or map. Wheel slip can cause errors; verify visually.'}
