"""Loopback HTTP contract tests using a mock backend, not a real VLM."""
from dataclasses import asdict
import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
from pai_robot.ollama_client import InvalidToolResponse, OllamaClient, DECISION_SCHEMA, parse_decision
from pai_robot.images import png_from_image
from pai_robot.navigation import Goal, GOAL_SCHEMA
from test_core import call

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('test_local_proxy', ROOT / 'tools/ollama_proxy.py')
proxy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(proxy)


def decision(tool='move'):
    action = call(tool)
    return {'observation': action.observation, 'candidate': None,
            'routes': {k:v for k,v in action.routes.items() if k != 'backward'},
            'action': {k:getattr(action,k) for k in ('tool','distance_m','direction','angle_deg')},
            'reason': action.next_subgoal, 'memory': action.memory}


class Backend(BaseHTTPRequestHandler):
    received = []
    raw = None
    done = True

    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'{"models":[{"name":"qwen3.5:4b-q8_0"}]}')

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        Backend.received.append(body)
        raw = self.raw
        if raw is None:
            raw = json.dumps(decision())
        self.send_response(200)
        self.end_headers()
        self.wfile.write(json.dumps({'done': self.done, 'message': {'content': raw}}).encode())


class HTTP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.token = 'a-test-token-of-more-than-24-chars'
        cls.backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
        handler = proxy.make_handler(cls.token, f'http://127.0.0.1:{cls.backend.server_port}')
        handler.log_message = lambda *args: None
        cls.proxy = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        cls.threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in (cls.backend, cls.proxy)]
        for thread in cls.threads:
            thread.start()
        cls.url = f'http://127.0.0.1:{cls.proxy.server_port}'

    @classmethod
    def tearDownClass(cls):
        for server in (cls.backend, cls.proxy):
            server.shutdown()
            server.server_close()
        for thread in cls.threads:
            thread.join()

    def request(self, path, headers=None, body=None):
        return build_opener(ProxyHandler({})).open(Request(self.url + path, headers=headers or {}, data=body), timeout=3)

    def client(self):
        with patch.dict(os.environ, {'OLLAMA_TOKEN': self.token, 'OLLAMA_MODEL': 'qwen3.5:4b-q8_0'}):
            return OllamaClient(self.url)

    def test_unauthenticated_rejected(self):
        with self.assertRaises(HTTPError) as ctx:
            self.request('/api/tags')
        self.assertEqual(ctx.exception.code, 401)

    def test_model_download_endpoint_rejected(self):
        with self.assertRaises(HTTPError) as ctx:
            self.request('/api/pull', {'Authorization': 'Bearer ' + self.token}, b'{}')
        self.assertEqual(ctx.exception.code, 403)

    def test_browser_origin_rejected(self):
        with self.assertRaises(HTTPError) as ctx:
            self.request('/api/tags', {'Authorization': 'Bearer ' + self.token, 'Origin': 'https://example.invalid'})
        self.assertEqual(ctx.exception.code, 403)

    def test_tags(self):
        with self.request('/api/tags', {'Authorization': 'Bearer ' + self.token}) as response:
            self.assertIn('models', json.load(response))

    def test_one_request_contains_image_goal_history_and_returns_observation_action(self):
        png = png_from_image(bytes([255, 0, 0]), 1, 1, 3, 'rgb8')
        context = {'instruction': '赤い箱を探して', 'fixed_goal': asdict(Goal('approach', 'red box', 'red', 'box', 1.0)),
                   'camera_and_robot': {'width': 320}, 'recent_tool_results': [{'tool': 'turn'}]}
        before = len(Backend.received)
        action, raw = self.client().decide(png, context)
        self.assertEqual(len(Backend.received)-before, 1)
        self.assertEqual(action, call())
        body = Backend.received[-1]
        self.assertEqual(json.loads(body['messages'][1]['content']), context)
        self.assertTrue(body['messages'][1]['images'][0].startswith('iVBOR'))
        self.assertEqual(body['format'], DECISION_SCHEMA)
        self.assertEqual(json.loads(raw), decision())
        self.assertFalse(body['stream'])
        self.assertNotIn('tools', body)

    def test_candidate_flags_are_derived_not_overridden_by_model(self):
        value = decision('finish')
        value['candidate'] = {'color': 'golden brown', 'shape': 'ball', 'position': 'center', 'ground_contact_px': [160, 195]}
        goal = asdict(Goal('approach', 'red sphere', 'red', 'sphere', 1.0))
        result = parse_decision(json.dumps(value), goal)
        self.assertEqual(result.candidate_color, 'brown')
        self.assertEqual(result.candidate_shape, 'sphere')
        self.assertFalse(result.target_visible)
        self.assertFalse(result.target_reached)
        value['target_reached'] = True
        with self.assertRaises(ValueError):
            parse_decision(json.dumps(value), goal)

    def test_routes_and_backward_visibility_are_not_ambiguous(self):
        value = decision()
        value['routes'] = {'left': 'clear', 'forward': 'blocked', 'right': 'unknown'}
        result = parse_decision(json.dumps(value), {})
        self.assertEqual(result.routes['forward'], 'blocked')
        self.assertEqual(result.routes['backward'], 'unknown')
        for routes in ([], {}, {'forward': 'clear'}, {**value['routes'], 'backward': 'clear'},
                       {**value['routes'], 'forward': 'maybe'}):
            with self.subTest(routes=routes), self.assertRaises(ValueError):
                parse_decision(json.dumps({**value, 'routes': routes}), {})

    def test_malformed_candidate_cannot_escape_validation(self):
        value = decision()
        for candidate in ({}, {'color': 'red', 'shape': 'sphere', 'position': 'center', 'ground_contact_px': [True, 160]},
                          {'color': 'red', 'shape': 'sphere', 'position': 'up', 'ground_contact_px': []}):
            with self.assertRaises(ValueError):
                parse_decision(json.dumps({**value, 'candidate': candidate}), {})

    def test_incomplete_model_response_rejected(self):
        with patch.object(Backend, 'done', False), self.assertRaises(ValueError):
            self.client().decide(png_from_image(bytes(3), 1, 1, 3, 'rgb8'), {})

    def test_mission_interpretation_has_no_scene_image_or_tool_history(self):
        goal = Goal('approach', 'red sphere', 'red', 'sphere', 1.0)
        context = {'task': 'interpret_mission', 'instruction': '赤い球を探して',
                   'arrival_policy': {'default_max_clearance_m': 1.0},
                   'recent_tool_results': [{'tool': 'move', 'distance_m': 5}]}
        with patch.object(Backend, 'raw', json.dumps(asdict(goal))):
            result, _ = self.client().decide(png_from_image(bytes(3), 1, 1, 3, 'rgb8'), context)
        self.assertEqual(result, goal)
        body = Backend.received[-1]
        self.assertEqual(body['format'], GOAL_SCHEMA)
        self.assertNotIn('images', body['messages'][1])
        self.assertNotIn('recent_tool_results', body['messages'][1]['content'])

    def test_invalid_model_action_rejected(self):
        with patch.object(Backend, 'raw', '{"tool":"shell"}'), self.assertRaises(InvalidToolResponse) as ctx:
            self.client().decide(png_from_image(bytes(3), 1, 1, 3, 'rgb8'), {})
        self.assertEqual(ctx.exception.raw, '{"tool":"shell"}')

    def test_reject_other_model(self):
        body = json.dumps({'model': 'other', 'stream': False}).encode()
        with self.assertRaises(HTTPError) as ctx:
            self.request('/api/chat', {'Authorization': 'Bearer ' + self.token}, body)
        self.assertEqual(ctx.exception.code, 400)

    def test_bad_png_rejected_before_network(self):
        with self.assertRaises(ValueError):
            self.client().decide(b'not png', {})
