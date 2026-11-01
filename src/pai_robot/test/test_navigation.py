"""Semantic arrival gates, geometric range and exploration progress."""
from dataclasses import asdict
import io
import json
import math
from pathlib import Path
import unittest
from PIL import Image, ImageDraw

from pai_robot.agent import CameraAgent, parse_action
from pai_robot.motion import OdometrySample
from pai_robot.navigation import GEOMETRY, Goal, SearchProgress, parse_goal, check_camera_profile
from test_core import call


def frame(color='gray', size=(320, 240)):
    output = io.BytesIO()
    image = Image.new('RGB', size, color)
    ImageDraw.Draw(image).rectangle((130, 80, 190, 195), fill='red')
    image.save(output, format='PNG')
    return output.getvalue()


class GoalAndArrival(unittest.TestCase):
    def agent(self, shape='sphere'):
        agent = CameraAgent('赤い球を探して近づいて', 0)
        agent.lock_goal(Goal('approach', '赤い球', 'red', shape, 1.0))
        return agent

    def present(self, agent, action):
        agent.observe_frame(frame())
        agent.begin_inference()
        agent.accept(action, agent.inference_calls)

    def test_orange_ball_cannot_finish_or_claim_red_target(self):
        for tool in ('finish', 'observe'):
            with self.subTest(tool=tool):
                agent = self.agent()
                self.present(agent, call(tool, candidate_color='orange', candidate_shape='sphere',
                                         target_visible=True, ground_contact_px=[160, 200]))
                self.assertEqual(agent.state, 'SETTLING')
                self.assertEqual(agent.history[-1]['status'], 'rejected')
                self.assertIn('orange sphere', agent.history[-1]['reason'])
                self.assertEqual(agent.goal.color, 'red')

    def test_color_shape_and_uncertainty_all_matter(self):
        for color, shape in (('red', 'cylinder'), ('unknown', 'sphere'), ('red', 'unknown'), ('none', 'none')):
            agent = self.agent()
            self.present(agent, call('finish', candidate_color=color, candidate_shape=shape))
            self.assertFalse(agent.assessment['candidate_matches_goal'])
            self.assertNotEqual(agent.state, 'DONE')

    def test_finish_requires_two_fresh_near_matching_observations(self):
        agent = self.agent()
        action = call('finish', candidate_shape='sphere')
        self.present(agent, action)
        self.assertNotEqual(agent.state, 'DONE')
        self.assertTrue(agent.assessment['within_arrival_distance'])
        # Re-evaluating the same captured frame cannot satisfy the second observation.
        agent.begin_inference()
        agent.accept(action, 2)
        self.assertNotEqual(agent.state, 'DONE')
        self.present(agent, action)
        self.assertEqual(agent.state, 'DONE')
        self.assertTrue(agent.assessment['confirmed_in_two_frames'])
        self.assertFalse(agent.assessment['independently_verified'])

    def test_far_hidden_or_clipped_ground_boundary_cannot_finish(self):
        for point in ([], [160, 110], [160, 135], [160, 239], [321, 200]):
            agent = self.agent()
            for _ in range(2):
                self.present(agent, call('finish', candidate_shape='sphere', ground_contact_px=point))
            self.assertNotEqual(agent.state, 'DONE', point)

    def test_mismatch_breaks_confirmation_sequence(self):
        agent = self.agent()
        self.present(agent, call('finish', candidate_shape='sphere'))
        self.present(agent, call('observe', candidate_color='orange', candidate_shape='sphere'))
        self.present(agent, call('finish', candidate_shape='sphere'))
        self.assertNotEqual(agent.state, 'DONE')

    def test_near_target_overrides_motion_and_auto_finishes_without_model_finish(self):
        for tool in ('move', 'turn', 'observe'):
            with self.subTest(tool=tool):
                agent = self.agent()
                proposal = call(tool, candidate_color='red', candidate_shape='sphere',
                                target_visible=True, ground_contact_px=[160, 195])
                self.present(agent, proposal)
                self.assertEqual(agent.state, 'SETTLING')
                self.assertIsNone(agent.motion)
                self.assertTrue(agent.assessment['arrival_confirmation_pending'])
                self.assertEqual(agent.history[-1]['tool_call']['tool'], 'observe')
                self.assertEqual(agent.history[-1]['proposed_tool_call']['tool'], tool)
                self.assertEqual(agent.context()['recent_tool_results'][-1]['tool'], 'observe')
                self.assertEqual(agent.advance(1, OdometrySample(1, 0, 0, 0)), (0, 0))
                self.present(agent, proposal)
                self.assertEqual(agent.state, 'DONE')
                self.assertEqual(agent.completion_source, 'camera_arrival_policy')
                self.assertEqual(agent.history[-1]['tool_call']['tool'], 'finish')
                self.assertEqual(agent.advance(2, OdometrySample(2, 0, 0, 0)), (0, 0))

    def test_lost_boundary_holds_then_stops_without_reusing_old_distance_as_success(self):
        agent = self.agent()
        self.present(agent, call('finish', candidate_shape='sphere'))
        for index in range(3):
            self.present(agent, call('move', candidate_color='red', candidate_shape='sphere',
                                     ground_contact_px=[160, 239]))
            self.assertIsNone(agent.motion)
            self.assertIsNone(agent.assessment['distance_estimate'])
            self.assertFalse(agent.assessment['confirmed_in_two_frames'])
            self.assertEqual(agent.state, 'STOPPED' if index == 2 else 'SETTLING')
        self.assertEqual(agent.history[-1]['status'], 'arrival_unconfirmed')
        self.assertIn('Arrival is unconfirmed', agent.reason)
        self.assertIsNone(agent.completion_source)

    def test_reused_missing_frame_does_not_exhaust_fresh_recheck_budget(self):
        agent = self.agent()
        self.present(agent, call('finish', candidate_shape='sphere'))
        action = call('observe')
        self.present(agent, action)
        for _ in range(4):
            agent.begin_inference()
            agent.accept(action, 3)
        self.assertEqual(agent.state, 'SETTLING')
        self.assertEqual(agent.arrival_hold['unconfirmed_frames'], 1)

    def test_confirmation_recovers_after_bad_point_with_two_new_valid_frames(self):
        agent = self.agent()
        action = call('finish', candidate_shape='sphere')
        self.present(agent, action)
        self.present(agent, call('observe', candidate_color='red', candidate_shape='sphere'))
        self.present(agent, action)
        self.assertEqual(agent.state, 'SETTLING')
        self.present(agent, action)
        self.assertEqual(agent.state, 'DONE')

    def test_fresh_far_estimate_releases_hold_and_allows_approach(self):
        agent = self.agent()
        self.present(agent, call('finish', candidate_shape='sphere'))
        output = io.BytesIO()
        image = Image.new('RGB', (320, 240), 'gray')
        ImageDraw.Draw(image).rectangle((130, 80, 190, 150), fill='red')
        image.save(output, format='PNG')
        agent.observe_frame(output.getvalue())
        agent.begin_inference()
        agent.advance(2, OdometrySample(2, 0, 0, 0))
        agent.accept(call('move', candidate_color='red', candidate_shape='sphere',
                          ground_contact_px=[160, 150]), 2)
        self.assertEqual(agent.state, 'EXECUTING')
        self.assertIsNone(agent.arrival_hold)
        self.assertIsNone(agent.previous_near_frame)

    def test_explicit_stop_is_respected_during_arrival_hold(self):
        agent = self.agent()
        self.present(agent, call('finish', candidate_shape='sphere'))
        self.present(agent, call('stop', candidate_color='red', candidate_shape='sphere',
                                 ground_contact_px=[160, 195]))
        self.assertEqual(agent.state, 'STOPPED')
        self.assertIsNone(agent.completion_source)

    def test_new_mission_does_not_inherit_near_hold(self):
        old = self.agent()
        self.present(old, call('finish', candidate_shape='sphere'))
        self.assertIsNotNone(old.arrival_hold)
        self.assertIsNone(self.agent().arrival_hold)

    def test_goal_lock_is_immutable_and_new_mission_starts_empty(self):
        agent = self.agent()
        with self.assertRaises(ValueError):
            agent.lock_goal(Goal('approach', 'orange sphere', 'orange', 'sphere', 1.0))
        self.assertIsNone(CameraAgent('new mission', 0).goal)

    def test_goal_response_cannot_actuate(self):
        agent = CameraAgent('赤い球', 0)
        agent.begin_inference()
        agent.accept(Goal('approach', 'red sphere', 'red', 'sphere', 1.0), .5)
        self.assertEqual(agent.state, 'SETTLING')
        self.assertIsNone(agent.motion)
        self.assertEqual(agent.actions, 0)

    def test_goal_contract_rejects_nonfinite_and_unknown_targets(self):
        valid = asdict(self.agent().goal)
        for extra in ({'max_clearance_m': float('nan')}, {'max_clearance_m': 0},
                      {'max_clearance_m': True}, {'color': 'unknown'}, {'shape': 'none'}):
            with self.assertRaises(ValueError):
                parse_goal(json.dumps({**valid, **extra}))


class CameraAndSearch(unittest.TestCase):
    def test_floor_ray_recovers_known_distance(self):
        c = GEOMETRY.context()
        for distance in (.5, 1., 2.):
            x = distance-GEOMETRY.camera_x_m+GEOMETRY.robot_front_x_m
            v = c['cy_px']+c['fy_px']*GEOMETRY.camera_height_m/x
            estimate = GEOMETRY.clearance([c['cx_px'], v], (320, 240))
            self.assertAlmostEqual(estimate['clearance_m'], distance, places=3)
        self.assertIsNone(GEOMETRY.clearance([160, 195], (640, 480)))

    def test_ground_point_is_snapped_to_edge_and_blank_floor_is_rejected(self):
        png = frame()
        result = GEOMETRY.clearance([160, 215], (320, 240), png)
        self.assertIsNotNone(result)
        self.assertLess(abs(result['used_point_px'][1]-195), 3)
        self.assertTrue(result['image_boundary_checked'])
        self.assertIsNone(GEOMETRY.clearance([260, 215], (320, 240), png))

    def test_recessed_camera_distance_is_relative_to_vehicle_front(self):
        # This center ray hits the floor 1 m ahead of the lens. The front of
        # the body is 5 cm ahead of the lens, so the surface gap is 0.95 m.
        result = GEOMETRY.clearance([159.5, 175.62912436779912], (320, 240))
        self.assertAlmostEqual(result['clearance_m'], .95, places=4)

    def test_world_camera_matches_calibration_and_body_width(self):
        import xml.etree.ElementTree as ET
        for path in (Path(__file__).resolve().parents[1]/'worlds').glob('*.sdf'):
            check_camera_profile(path.read_text())
            robot = ET.parse(path).getroot().find(".//model[@name='visual_bot']")
            camera = robot.find(".//sensor[@type='camera']")
            self.assertEqual(int(camera.findtext('camera/image/width')), GEOMETRY.width)
            self.assertEqual(int(camera.findtext('camera/image/height')), GEOMETRY.height)
            self.assertAlmostEqual(float(camera.findtext('camera/horizontal_fov')), GEOMETRY.horizontal_fov_rad)
            pose = list(map(float, camera.findtext('pose').split()))
            self.assertEqual(pose, [GEOMETRY.camera_x_m, 0, GEOMETRY.camera_height_m, 0, GEOMETRY.camera_pitch_rad, 0])
            self.assertEqual(float(camera.findtext('camera/clip/near')), GEOMETRY.camera_near_clip_m)
            body = robot.find("link[@name='base_link']/collision")
            body_x = float(body.findtext('pose').split()[0])
            half_length = float(body.findtext('geometry/box/size').split()[0])/2
            self.assertGreater(body_x+half_length-pose[0], GEOMETRY.camera_near_clip_m)

    def test_launch_rejects_stale_camera_pose_or_clip_profile(self):
        import xml.etree.ElementTree as ET
        path = Path(__file__).resolve().parents[1]/'worlds/hidden_search.sdf'
        for field, value in (('pose', '0.16 0 0.24 0 0 0'), ('camera/clip/near', '0.03')):
            root = ET.fromstring(path.read_text())
            root.find(".//model[@name='visual_bot']//sensor[@type='camera']/"+field).text = value
            with self.assertRaises(ValueError):
                check_camera_profile(ET.tostring(root, encoding='unicode'))

    def test_static_views_produce_progress_feedback_and_translation_resets_scan(self):
        search = SearchProgress()
        search.observe_frame(frame())
        search.observe_frame(frame())
        self.assertTrue(search.context()['feedback'])
        for _ in range(4):
            search.motion_completed(call('turn', direction='right', angle_deg=90))
            search.observe_frame(frame())
        self.assertEqual(len(search.views), 4)
        self.assertGreaterEqual(len(search.context()['feedback']), 2)
        search.motion_completed(call('move'))
        self.assertEqual(search.views, [])
        self.assertEqual(search.observations_since_translation, 0)

    def test_unrequested_candidate_is_remembered_without_replacing_goal(self):
        agent = CameraAgent('red ball', 0)
        agent.lock_goal(Goal('approach', 'red ball', 'red', 'sphere', 1.0))
        agent.observe_frame(frame())
        agent.begin_inference()
        agent.accept(call('observe', candidate_color='orange', candidate_shape='sphere',
                          routes={'left': 'clear', 'forward': 'blocked', 'right': 'unknown', 'backward': 'unknown'},
                          next_subgoal='Inspect floor to the left'), 1)
        context = agent.context()
        self.assertEqual(context['fixed_goal']['color'], 'red')
        self.assertFalse(context['search_progress']['recent_candidates'][-1]['matches_fixed_goal'])
        self.assertEqual(context['search_progress']['last_routes']['left'], 'clear')
        self.assertEqual(context['search_progress']['last_routes']['forward'], 'blocked')

    def test_action_rejects_malformed_visual_evidence(self):
        for extra in ({'ground_contact_px': [160]}, {'ground_contact_px': [True, 180]},
                      {'ground_contact_px': [160, float('nan')]}, {'routes': {'forward': 'clear'}},
                      {'candidate_color': 'red/orange'}, {'candidate_shape': 'maybe_sphere'}):
            with self.assertRaises(ValueError):
                parse_action(json.dumps({**asdict(call()), **extra}))

    def test_blocked_or_unknown_translation_is_rejected_before_actuation(self):
        for distance, direction in ((.5, 'forward'), (-.5, 'backward')):
            for state in ('blocked', 'unknown'):
                agent = CameraAgent('Find a red ball', 0)
                agent.lock_goal(Goal('approach', 'red sphere', 'red', 'sphere', 1.0))
                agent.begin_inference()
                routes = {d: 'clear' for d in ('left', 'forward', 'right', 'backward')}
                routes[direction] = state
                agent.accept(call('move', distance_m=distance, routes=routes), .5)
                self.assertEqual(agent.state, 'SETTLING')
                self.assertIsNone(agent.motion)
                self.assertEqual(agent.actions, 0)
                self.assertIn(direction, agent.history[-1]['reason'])

    def test_unknown_translation_does_not_prevent_observing_or_stopping(self):
        for tool in ('observe', 'stop'):
            agent = CameraAgent('Inspect', 0)
            agent.lock_goal(Goal('inspect', 'Inspect', 'unspecified', 'unspecified', 1.0))
            agent.begin_inference()
            agent.accept(call(tool, routes={d: 'unknown' for d in ('left', 'forward', 'right', 'backward')}), .5)
            self.assertEqual(agent.actions, 1)

    def test_one_model_call_is_charged_for_each_decision_and_rejection(self):
        agent = CameraAgent('Inspect', 0, max_actions=3)
        self.assertTrue(agent.begin_inference())
        agent.accept(Goal('inspect', 'Inspect', 'unspecified', 'unspecified', 1.0), .5)
        self.assertTrue(agent.begin_inference())
        agent.reject('Malformed decision', 1)
        self.assertEqual(agent.inference_calls, 2)
        self.assertTrue(agent.begin_inference())
        agent.reject('Malformed decision', 2)
        self.assertFalse(agent.begin_inference())

    def test_evaluator_uses_front_to_surface_distance(self):
        import importlib.util
        path = Path(__file__).resolve().parents[3]/'tools/trial_evaluation.py'
        spec = importlib.util.spec_from_file_location('eval_clearance_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        obj = {'pose': [3,0,0,0,0,0], 'geometry': '<geometry><sphere><radius>0.25</radius></sphere></geometry>'}
        self.assertAlmostEqual(module.surface_clearance(0,0,0,obj), 2.6)
        obj['geometry'] = '<geometry><box><size>0.4 0.8 1</size></box></geometry>'
        self.assertAlmostEqual(module.surface_clearance(0,0,0,obj), 2.65)
        obj['pose'][-1] = math.pi/2
        self.assertAlmostEqual(module.surface_clearance(0,0,0,obj), 2.45)
