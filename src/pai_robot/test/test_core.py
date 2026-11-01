"""Pure contracts and execution tests; no learned policy or simulation claims."""
from dataclasses import asdict
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from PIL import Image
from pai_robot.agent import ACTION_SCHEMA, CameraAgent, parse_action, parse_mission
from pai_robot.navigation import Goal
from pai_robot.images import png_from_image
from pai_robot.safety import guard_velocity
from pai_robot.motion import Motion, OdometrySample, wrap


def arrival_png():
    image = Image.new('RGB', (320, 240), 'gray')
    image.paste('red', (130, 80, 190, 195))
    output = io.BytesIO()
    image.save(output, format='PNG')
    return output.getvalue()


def call(tool='move', **kwargs):
    value = dict(tool=tool, distance_m=0, direction='none', angle_deg=0,
                 observation='Visible floor ahead', memory='Looking for the red box',
                 target_visible=False, target_reached=False, candidate_color='none', candidate_shape='none',
                 ground_contact_px=[], routes={'left': 'unknown', 'forward': 'clear', 'right': 'unknown', 'backward': 'unknown'},
                 next_subgoal='Look for a clear route')
    if tool == 'move':
        value.update(distance_m=0.1)
    if tool == 'turn':
        value.update(direction='left', angle_deg=30)
    if tool == 'finish':
        value.update(target_visible=True, target_reached=True, candidate_color='red',
                     candidate_shape='box', ground_contact_px=[160, 195])
    value.update(kwargs)
    return parse_action(json.dumps(value))


class Contract(unittest.TestCase):
    def test_generation_schema_separates_motion_arguments(self):
        branches = {s['properties']['tool']['const']: s['properties'] for s in ACTION_SCHEMA['anyOf']}
        self.assertEqual(branches['move']['angle_deg'], {'const': 0})
        self.assertEqual(branches['turn']['distance_m'], {'const': 0})
        self.assertEqual(branches['turn']['direction']['enum'], ['left', 'right'])
        self.assertEqual(branches['turn']['angle_deg']['minimum'], 0)
        for tool in ('move', 'observe', 'finish', 'stop'):
            self.assertEqual(branches[tool]['direction'], {'const': 'none'})
        self.assertNotIn('speed_m_s', branches['move'])
        self.assertNotIn('angular_speed_rad_s', branches['turn'])
        self.assertEqual(branches['finish']['target_reached'], {'type': 'boolean'})

    def test_natural_language_is_preserved(self):
        self.assertEqual(parse_mission('{"instruction":"  壁の裏の赤い球を探して  "}'), '壁の裏の赤い球を探して')
        for value in ({'instruction': ''}, {'instruction': 8}, {'instruction': 'x', 'coordinates': [1, 2]}, ['x']):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_mission(json.dumps(value))

    def test_amounts_have_no_old_distance_angle_or_six_second_limit(self):
        self.assertEqual(call(distance_m=1).distance_m, 1)
        self.assertEqual(call(distance_m=-.5).distance_m, -.5)
        self.assertEqual(call('turn', angle_deg=360).angle_deg, 360)
        self.assertEqual(call(distance_m=0).distance_m, 0)

    def test_reverse_and_explicit_turn_directions(self):
        self.assertEqual(call(distance_m=-.10).distance_m, -.1)
        self.assertEqual(call('turn', direction='left', angle_deg=60).signed_angle_deg, 60)
        self.assertEqual(call('turn', direction='right', angle_deg=60).signed_angle_deg, -60)

    def test_ambiguous_turns_and_legacy_signed_arguments_rejected(self):
        for kwargs in (dict(angle_deg=-30), dict(direction='none'), dict(direction='LEFT'),
                       dict(direction=1), dict(direction=['left'])):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                call('turn', **kwargs)
        old = asdict(call('turn'))
        del old['direction']
        with self.assertRaises(ValueError):
            parse_action(json.dumps(old))
        for tool in ('move', 'observe', 'finish', 'stop'):
            with self.subTest(tool=tool), self.assertRaises(ValueError):
                call(tool, direction='left')

    def test_untrusted_motion_rejected(self):
        invalid = [dict(distance_m=float('nan')), dict(distance_m=float('inf')),
                   dict(distance_m=10**400),
                   dict(distance_m=True), dict(distance_m='0.1'), dict(angle_deg=30),
                   dict(speed_m_s=.1),
                   dict(tool='shell'), dict(memory='x' * 801), dict(observation=''), dict(target_visible=1)]
        for kwargs in invalid:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                call(**kwargs)

    def test_no_motion_in_stationary_tools(self):
        for tool in ('observe', 'finish', 'stop'):
            self.assertEqual(call(tool).distance_m, 0)
            with self.assertRaises(ValueError):
                call(tool, distance_m=.1)

    def test_unknown_and_missing_fields_rejected(self):
        valid = asdict(call())
        for raw in (json.dumps([valid]), '```json\n' + json.dumps(valid) + '\n```',
                    json.dumps({**valid, 'code': 'anything'}), json.dumps({k: v for k, v in valid.items() if k != 'memory'})):
            with self.assertRaises(ValueError):
                parse_action(raw)

    def test_finish_flags_are_claims_not_proof(self):
        self.assertFalse(call('finish', target_reached=False).target_reached)


class Execution(unittest.TestCase):
    def setUp(self):
        self.agent = CameraAgent('Approach the red box', 0)
        self.agent.lock_goal(Goal('approach', 'red box', 'red', 'box', 1.0))
        self.agent.advance(.5, OdometrySample(.5, 0, 0, 0))
        self.assertTrue(self.agent.ready(.5, .5))
        self.agent.begin_inference()

    def test_direction_drives_yaw_and_is_preserved_in_model_history(self):
        for direction, sign in (('left', 1), ('right', -1)):
            with self.subTest(direction=direction):
                self.setUp()
                self.agent.accept(call('turn', direction=direction, angle_deg=30), .5)
                v, w = self.agent.advance(.6, OdometrySample(.6, 0, 0, 0))
                self.assertEqual(v, 0)
                self.assertGreater(sign*w, 0)
                for sim in (.7, .8, .9, 1.0):
                    self.agent.advance(sim, OdometrySample(sim, 0, 0, sign*math.radians(30)))
                result = self.agent.context()['recent_tool_results'][-1]
                self.assertEqual(result['status'], 'completed')
                self.assertEqual(result['direction'], direction)
                self.assertEqual(result['angle_deg'], 30)

    def test_stop_observe_think_execute_cycle(self):
        self.assertEqual(self.agent.advance(.6, OdometrySample(.6, 0, 0, 0)), (0, 0))
        self.agent.accept(call(distance_m=.10), .6)
        self.assertGreater(self.agent.advance(.7, OdometrySample(.7, .02, 0, 0))[0], 0)
        # Position feedback, not elapsed time, determines completion.
        for sim in (.8, .9, 1.0, 1.1):
            self.assertEqual(self.agent.advance(sim, OdometrySample(sim, .1, 0, 0)), (0, 0))
        self.assertEqual(self.agent.state, 'SETTLING')
        self.agent.advance(1.6, OdometrySample(1.6, .1, 0, 0))
        self.assertFalse(self.agent.ready(1.6, 1.0))
        self.assertTrue(self.agent.ready(1.6, 1.6))
        result = self.agent.history[-1]
        self.assertAlmostEqual(result['odometry_distance_m'], .1)
        self.assertEqual(result['motion_feedback'], 'wheel_odometry')

    def test_no_progress_returns_failure_for_replanning(self):
        self.agent.accept(call(), .5)
        for step in range(1, 33):
            sim = .5 + step*.1
            self.agent.advance(sim, OdometrySample(sim, 0, 0, 0))
        self.assertEqual(self.agent.state, 'SETTLING')
        self.assertEqual(self.agent.history[-1]['status'], 'motion_failed')

    def test_stale_odometry_stops_motion(self):
        self.agent.accept(call(), .5)
        for sim in (.7, .9, 1.1):
            self.agent.advance(sim)
        self.assertEqual(self.agent.state, 'ERROR')
        self.assertIn('odometry', self.agent.reason)

    def test_long_control_gap_aborts_motion(self):
        self.agent.accept(call(), .5)
        self.assertEqual(self.agent.advance(.9), (0, 0))
        self.assertEqual(self.agent.state, 'ERROR')
        self.assertEqual(self.agent.history[-1]['status'], 'interrupted')

    def test_clock_reset_stops(self):
        self.agent.accept(call(), .5)
        self.assertEqual(self.agent.advance(.4), (0, 0))
        self.assertEqual(self.agent.state, 'ERROR')

    def test_model_memory_and_results_in_next_observation(self):
        self.agent.accept(call('observe', memory='Left route has a wall'), .5)
        context = self.agent.context()
        self.assertEqual(context['memory_from_previous_model_call'], 'Left route has a wall')
        self.assertEqual(context['recent_tool_results'][-1]['status'], 'observed')
        self.assertNotIn('memory', context['recent_tool_results'][-1])
        self.assertNotIn('observation', context['recent_tool_results'][-1])
        self.assertNotIn('odometry_distance_m', str(context))
        self.assertNotIn('odometry_angle_deg', str(context))
        self.assertNotIn('origin', context)
        self.assertNotIn('origin', context['recent_tool_results'][-1])

    def test_rejected_call_is_returned_and_counts_toward_budget(self):
        self.agent.max_actions = 1
        self.agent.reject('move requires angle_deg=0', .6)
        self.assertEqual(self.agent.state, 'SETTLING')
        self.assertEqual(self.agent.context()['recent_tool_results'][-1]['status'], 'rejected')
        self.assertFalse(self.agent.begin_inference())

    def test_camera_confirmation_finishes_and_latches_without_independent_verification(self):
        self.agent.search.image_size = (320, 240)
        self.agent.search.last_png = arrival_png()
        self.agent.search.frame_id = 2
        self.agent.previous_near_frame = 1
        self.agent.accept(call('finish'), .5)
        self.assertEqual(self.agent.state, 'DONE')
        self.assertEqual(self.agent.advance(3), (0, 0))
        self.assertIn('no independent distance/contact verification', self.agent.reason)
        self.assertEqual(self.agent.completion_source, 'camera_arrival_policy')
        with self.assertRaises(RuntimeError):
            self.agent.accept(call(), 3)

    def test_simulation_budget(self):
        self.agent.max_sim_seconds = 1
        self.assertEqual(self.agent.advance(1.1), (0, 0))
        self.assertEqual(self.agent.state, 'ERROR')

    def test_tool_call_budget(self):
        self.agent.max_actions = 1
        self.agent.accept(call('observe'), .5)
        self.assertFalse(self.agent.begin_inference())
        self.assertEqual(self.agent.state, 'ERROR')

    def test_halt_records_interrupted_command(self):
        self.agent.accept(call(), .5)
        self.agent.stop('HALTED', 'Operator', .7)
        self.assertEqual(self.agent.history[-1]['status'], 'interrupted')
        self.assertAlmostEqual(self.agent.history[-1]['odometry_distance_m'], 0)
        self.assertEqual(self.agent.advance(.8), (0, 0))


class Watchdog(unittest.TestCase):
    def guard(self, v=.1, w=.2, **kwargs):
        return guard_velocity(v, w, **{'command_age': .1, 'camera_age': .1, 'odom_age': .1, 'clock_age': .1, **kwargs})

    def test_allowed_robot_command_passes(self):
        self.assertEqual(self.guard(.07, -.3), (.07, -.3, 'ok'))

    def test_stale_inputs_stop(self):
        for key, age in (('command_age', .6), ('camera_age', 1.6), ('odom_age', .6), ('clock_age', .8), ('camera_age', -1)):
            with self.subTest(key=key):
                self.assertEqual(self.guard(**{key: age})[:2], (0, 0))

    def test_nonfinite_input_stops(self):
        for value in (float('inf'), float('nan')):
            self.assertEqual(self.guard(value)[:2], (0, 0))
            self.assertEqual(self.guard(clock_age=value)[:2], (0, 0))

    def test_velocity_limit(self):
        self.assertEqual(self.guard(.5, -2), (.12, -.7, 'velocity_limit'))


class OdometryControl(unittest.TestCase):
    def follow(self, tool, amount, yaw=0):
        x = y = 0.
        odom = OdometrySample(0, x, y, yaw)
        motion = Motion(tool, amount, odom, 0)
        v = w = 0.
        for step in range(1, 1001):
            dt, sim = .05, step*.05
            x += v*math.cos(yaw+w*dt/2)*dt
            y += v*math.sin(yaw+w*dt/2)*dt
            yaw = wrap(yaw+w*dt)
            odom = OdometrySample(sim, x, y, yaw, v, w)
            v, w, outcome = motion.update(sim, odom)
            if outcome:
                self.assertEqual(outcome, 'completed')
                self.assertEqual((v,w), (0,0))
                self.assertTrue(odom.stationary)
                return motion, sim
        self.fail('Closed-loop controller did not converge')

    def test_forward_reverse_at_rotated_heading(self):
        for distance in (.3, -.2, 1.0):
            with self.subTest(distance=distance):
                motion, elapsed = self.follow('move', distance, math.pi/2)
                self.assertAlmostEqual(motion.distance, distance, delta=.008)
                if distance == 1:
                    self.assertGreater(elapsed, 6)

    def test_signed_turns_unwrap_and_full_turn(self):
        for angle in (math.pi/2, -math.pi/2, math.tau, -math.tau):
            with self.subTest(angle=angle):
                motion, _ = self.follow('turn', angle, math.radians(170))
                self.assertAlmostEqual(motion.angle, angle, delta=math.radians(1))

    def test_zero_motion_is_stopped_noop(self):
        motion, _ = self.follow('move', 0)
        self.assertEqual(motion.distance, 0)

    def test_target_pose_does_not_complete_while_wheels_are_moving(self):
        motion = Motion('move', .1, OdometrySample(0,0,0,0),0)
        for i in range(1,11):
            self.assertIsNone(motion.update(i*.05, OdometrySample(i*.05,.1,0,0,.1,0))[2])


class CameraAndWorld(unittest.TestCase):
    def test_padded_rgb_and_png_roundtrip(self):
        pixels = bytes([255, 0, 0, 0, 255, 0, 99, 99])
        decoded = Image.open(io.BytesIO(png_from_image(pixels, 2, 1, 8, 'rgb8')))
        self.assertEqual([decoded.getpixel((x, 0)) for x in range(2)], [(255, 0, 0), (0, 255, 0)])

    def test_all_encodings(self):
        for encoding, data in [('rgb8', [255, 0, 0]), ('rgba8', [255, 0, 0, 255]),
                               ('bgr8', [0, 0, 255]), ('bgra8', [0, 0, 255, 255])]:
            decoded = Image.open(io.BytesIO(png_from_image(bytes(data), 1, 1, len(data), encoding)))
            self.assertEqual(decoded.getpixel((0, 0)), (255, 0, 0))

    def test_bad_images(self):
        for args in ((b'', 1, 1, 3, 'rgb8'), (b'123', 1, 1, 2, 'rgb8'), (b'123', 1, 1, 3, 'mono8')):
            with self.assertRaises(ValueError):
                png_from_image(*args)

    def test_worlds_reproducible_and_single_rgb_sensor(self):
        root = Path(__file__).resolve().parents[3]
        spec = importlib.util.spec_from_file_location('world_builder', root / 'tools/generate_world.py')
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        with tempfile.TemporaryDirectory() as temp:
            for hidden in (False, True):
                path = builder.make(hidden=hidden, output_dir=temp)
                self.assertEqual(path.read_bytes(), (root / 'src/pai_robot/worlds' / path.name).read_bytes())
                sdf = ET.parse(path)
                sensors = sdf.findall('.//sensor')
                self.assertEqual([(s.get('name'), s.get('type')) for s in sensors], [('camera', 'camera')])
                self.assertIsNone(sdf.find('.//plugin/odom_topic'))
                self.assertEqual(sdf.findtext('.//plugin/odom_publish_frequency'), '20')
                self.assertFalse(sdf.findall('.//include'))
