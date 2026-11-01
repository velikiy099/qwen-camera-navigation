"""Mission contracts, camera geometry and observation progress; no world lookup."""
from dataclasses import asdict, dataclass
from io import BytesIO
import math
import json
from PIL import Image

COLORS = ('red', 'orange', 'yellow', 'yellow_green', 'green', 'blue', 'purple',
          'brown', 'black', 'white', 'gray', 'unknown', 'none')
SHAPES = ('sphere', 'box', 'cylinder', 'unknown', 'none')
ROUTE_DIRECTIONS = ('left', 'forward', 'right', 'backward')
ROUTE_STATES = ('clear', 'blocked', 'unknown')
DEFAULT_CLEARANCE_M = 1.0


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class Goal:
    kind: str
    description: str
    color: str
    shape: str
    max_clearance_m: float


GOAL_SCHEMA = {'type': 'object', 'properties': {
    'kind': {'type': 'string', 'enum': ['approach', 'inspect', 'unsupported']},
    'description': {'type': 'string', 'minLength': 1, 'maxLength': 300},
    'color': {'type': 'string', 'enum': list(COLORS) + ['unspecified']},
    'shape': {'type': 'string', 'enum': list(SHAPES) + ['unspecified']},
    'max_clearance_m': {'type': 'number', 'exclusiveMinimum': 0}},
    'required': ['kind', 'description', 'color', 'shape', 'max_clearance_m'],
    'additionalProperties': False}


def parse_goal(raw):
    value = json.loads(raw)
    if not isinstance(value, dict) or set(value) != set(GOAL_SCHEMA['required']):
        raise ValueError('One complete mission goal is required')
    for key in ('kind', 'color', 'shape'):
        if value[key] not in GOAL_SCHEMA['properties'][key]['enum']:
            raise ValueError(f'Invalid goal {key}')
    if not isinstance(value['description'], str) or not 1 <= len(value['description'].strip()) <= 300:
        raise ValueError('Invalid goal description')
    distance = value['max_clearance_m']
    if not finite_number(distance) or distance <= 0:
        raise ValueError('Arrival clearance must be a positive finite distance')
    if value['kind'] == 'approach' and (value['color'] in ('unknown', 'none') or value['shape'] in ('unknown', 'none')):
        raise ValueError('Do not invent an ambiguous target: use unsupported or unspecified attributes')
    return Goal(**value)


@dataclass(frozen=True)
class CameraGeometry:
    width: int = 320
    height: int = 240
    horizontal_fov_rad: float = 1.2
    camera_x_m: float = .10
    camera_height_m: float = .24
    camera_pitch_rad: float = 0.0
    camera_near_clip_m: float = .005
    camera_far_clip_m: float = 10.0
    robot_front_x_m: float = .15
    robot_width_m: float = .33
    robot_length_m: float = .30

    def context(self):
        f = self.width / (2 * math.tan(self.horizontal_fov_rad / 2))
        return {**asdict(self), 'fx_px': f, 'fy_px': f,
                'cx_px': (self.width - 1) / 2, 'cy_px': (self.height - 1) / 2,
                'pixel_coordinates': 'u right, v down; origin is top-left; use original image pixels',
                'distance_reference': 'horizontal distance from vehicle front to target near surface',
                'assumptions': 'level robot, flat ground, undistorted pinhole camera; no depth sensor'}

    def clearance(self, point, image_size, png=None):
        """Intersect the VLM-selected ground-boundary ray with the flat floor.

        This estimates distance to the selected floor point, NOT measured depth.
        A sphere's floor contact can be behind its near surface (conservative).
        Incorrect model localization can still produce incorrect distances.
        """
        if image_size != (self.width, self.height) or len(point) != 2:
            return None
        u, v = point
        if not (1 <= u < self.width-1 and 1 <= v < self.height-1):
            return None  # A clipped boundary is not visible ground contact.
        original_point = list(point)
        if png is not None:
            point = self.refine_ground_boundary(png, point)
            if point is None:
                return None
            u, v = point
        c = self.context()
        down = (v-c['cy_px']) / c['fy_px']
        forward = math.cos(self.camera_pitch_rad)-down*math.sin(self.camera_pitch_rad)
        vertical = math.sin(self.camera_pitch_rad)+down*math.cos(self.camera_pitch_rad)
        if vertical <= 0 or forward <= 0:
            return None  # At/above horizon, or ray points behind camera.
        scale = self.camera_height_m / vertical
        x = self.camera_x_m + scale*forward - self.robot_front_x_m
        y = -scale*(u-c['cx_px'])/c['fx_px']
        if x <= 0:
            return None
        return {'clearance_m': round(math.hypot(x, y), 4),
                'bearing_deg': round(math.degrees(math.atan2(y, x)), 2),
                'model_point_px': original_point, 'used_point_px': [u, v],
                'image_boundary_checked': png is not None,
                'source': 'RGB ground-boundary point and flat-floor camera geometry',
                'independent_measurement': False}

    def refine_ground_boundary(self, png, point):
        """Local edge sanity check, not object/color detection or obstacle avoidance.

        For this flat, textureless-floor demo, snap a proposed contact to a nearby
        vertical contrast transition with floor-like pixels below it. Reject
        homogeneous floor points. 12/255 contrast and a 15%-height search window
        are image-processing tuning values, not confidence bounds.
        """
        from statistics import median
        with Image.open(BytesIO(png)) as source:
            image = source.convert('RGB')
            if image.size != (self.width, self.height):
                return None
            pixels = image.load()
            floor = [median(pixels[x, self.height-2][c] for x in range(self.width)) for c in range(3)]
            u, v = map(round, point)
            if not 2 <= u < self.width-2:
                return None
            radius = round(self.height*.15)
            edges = []
            for row in range(max(round((self.height-1)/2)+2, v-radius, 3), min(self.height-3, v+radius+1)):
                above = [sum(pixels[x, y][c] for x in range(u-1,u+2) for y in (row-2,row-1))/6 for c in range(3)]
                below = [sum(pixels[x, y][c] for x in range(u-1,u+2) for y in (row+1,row+2))/6 for c in range(3)]
                contrast = sum(abs(a-b) for a,b in zip(above,below))/3
                floor_error = sum(abs(a-b) for a,b in zip(below,floor))/3
                if contrast >= 12 and floor_error <= 12:
                    edges.append(row)
            return [u, min(edges, key=lambda row: abs(row-v))] if edges else None


GEOMETRY = CameraGeometry()


def check_camera_profile(world_text):
    """Check only robot camera calibration; never export environment objects."""
    import xml.etree.ElementTree as ET
    robot = ET.fromstring(world_text).find(".//model[@name='visual_bot']")
    sensor = robot.find(".//sensor[@type='camera']") if robot is not None else None
    if sensor is None:
        raise ValueError('The configured camera geometry requires a visual_bot RGB camera')
    values = [float(sensor.findtext('camera/image/width')), float(sensor.findtext('camera/image/height')),
              float(sensor.findtext('camera/horizontal_fov')), *map(float, sensor.findtext('pose').split()),
              float(sensor.findtext('camera/clip/near')), float(sensor.findtext('camera/clip/far'))]
    expected = [GEOMETRY.width, GEOMETRY.height, GEOMETRY.horizontal_fov_rad,
                GEOMETRY.camera_x_m, 0, GEOMETRY.camera_height_m, 0, GEOMETRY.camera_pitch_rad, 0,
                GEOMETRY.camera_near_clip_m, GEOMETRY.camera_far_clip_m]
    if len(values) != len(expected) or any(not math.isclose(a, b, abs_tol=1e-6) for a, b in zip(values, expected)):
        raise ValueError('Camera SDF and navigation.CameraGeometry differ; update calibration before using metric arrival checks')


class SearchProgress:
    """Command-relative viewing ledger, not a map or actual robot position."""
    def __init__(self):
        self.heading = 0.0
        self.views = []
        self.sightings = []
        self.last_pixels = None
        self.unchanged = 0
        self.observations_since_translation = 0
        self.last_plan = ''
        self.routes = {direction: 'unknown' for direction in ROUTE_DIRECTIONS}
        self.frame_id = 0
        self.image_size = None
        self.last_png = None

    def observe_frame(self, png):
        with Image.open(BytesIO(png)) as image:
            self.image_size = image.size
            pixels = bytes(image.convert('RGB').resize((16, 12)).tobytes())
        self.last_png = png
        # 2/255 mean change is a diagnostic tuning value, not a collision test.
        similar = self.last_pixels is not None and sum(abs(a-b) for a, b in zip(pixels, self.last_pixels))/len(pixels) <= 2
        self.unchanged = self.unchanged + 1 if similar else 0
        self.last_pixels = pixels
        self.frame_id += 1
        self.observations_since_translation += 1
        if all(abs((self.heading-h+180) % 360-180) > math.degrees(GEOMETRY.horizontal_fov_rad)/2 for h in self.views):
            self.views.append(round(self.heading, 1))

    def motion_completed(self, action):
        if action.tool == 'turn':
            self.heading = (self.heading + action.signed_angle_deg) % 360
        elif action.tool == 'move' and action.distance_m != 0:
            self.views = []
            self.observations_since_translation = 0

    def record(self, action, matched):
        self.last_plan = action.next_subgoal
        self.routes = dict(action.routes)
        if action.candidate_color != 'none':
            self.record_candidate(action.candidate_color, action.candidate_shape, matched)

    def record_candidate(self, color, shape, matched):
        item = {'frame': self.frame_id, 'command_heading_deg': round(self.heading, 1),
                'color': color, 'shape': shape, 'matches_fixed_goal': matched}
        if item not in self.sightings:
            self.sightings.append(item)
            self.sightings = self.sightings[-24:]

    def context(self):
        feedback = []
        if self.unchanged >= 1:
            feedback.append('The view changed little. Repeating observe does not explore a new place; choose a useful new view, visible clear route, or stop.')
        if len(self.views) >= 4:
            feedback.append('Several directions have already been viewed at this stop. Choose a visible clear route toward an unexplored place instead of repeating the scan.')
        return {'view_headings_at_current_stop_deg': self.views,
                'command_heading_deg': round(self.heading, 1),
                'heading_basis': 'sum of commanded turns, NOT measured pose; may drift',
                'observations_since_translation': self.observations_since_translation,
                'consecutive_similar_views': self.unchanged,
                'recent_candidates': self.sightings,
                'last_routes': self.routes,
                'next_subgoal': self.last_plan, 'feedback': feedback}
