"""Loopback HTTP tests with a MOCK model. Never a real VLM inference claim."""
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
from pai_robot.ollama_client import OllamaClient
from pai_robot.images import png_from_bgra

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('test_local_proxy',ROOT/'tools/ollama_proxy.py')
proxy=importlib.util.module_from_spec(spec); spec.loader.exec_module(proxy)
class Backend(BaseHTTPRequestHandler):
    received=[]
    def log_message(self,*args): pass
    def do_GET(self):
        self.send_response(200); self.end_headers(); self.wfile.write(b'{"models":[{"name":"qwen2.5vl:3b"}]}')
    def do_POST(self):
        body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        Backend.received.append(body)
        raw='{"visible_colors":["red"]}' if 'images' in body['messages'][0] else '{"target_color":"red"}'
        self.send_response(200); self.end_headers(); self.wfile.write(json.dumps({'message':{'content':raw}}).encode())
class HTTP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.token='a-test-token-of-more-than-24-chars'
        cls.backend=ThreadingHTTPServer(('127.0.0.1',0),Backend)
        handler=proxy.make_handler(cls.token,f'http://127.0.0.1:{cls.backend.server_port}')
        handler.log_message=lambda *args:None
        cls.proxy=ThreadingHTTPServer(('127.0.0.1',0),handler)
        cls.threads=[threading.Thread(target=s.serve_forever,daemon=True) for s in (cls.backend,cls.proxy)]
        for t in cls.threads:t.start()
        cls.url=f'http://127.0.0.1:{cls.proxy.server_port}'
    @classmethod
    def tearDownClass(cls):
        for s in (cls.backend,cls.proxy): s.shutdown(); s.server_close()
        for t in cls.threads:t.join()
    def request(self,path,headers=None,body=None):
        req=Request(self.url+path,headers=headers or {},data=body)
        return build_opener(ProxyHandler({})).open(req,timeout=3)
    def test_unauthenticated_rejected(self):
        with self.assertRaises(HTTPError) as ctx:self.request('/api/tags')
        self.assertEqual(ctx.exception.code,401)
    def test_model_download_endpoint_rejected(self):
        with self.assertRaises(HTTPError) as ctx:self.request('/api/pull',{'Authorization':'Bearer '+self.token},b'{}')
        self.assertEqual(ctx.exception.code,403)
    def test_browser_origin_rejected(self):
        with self.assertRaises(HTTPError) as ctx:self.request('/api/tags',{'Authorization':'Bearer '+self.token,'Origin':'https://example.invalid'})
        self.assertEqual(ctx.exception.code,403)
    def test_tags(self):
        with self.request('/api/tags',{'Authorization':'Bearer '+self.token}) as r:
            self.assertIn('models',json.load(r))
    def test_real_client_mock_backend(self):
        with patch.dict(os.environ,{'OLLAMA_TOKEN':self.token}):client=OllamaClient(self.url)
        colors,raw=client.recognize(png_from_bgra(bytes([0,0,255,255]),1,1))
        self.assertEqual(colors,{'red'})
        message=Backend.received[-1]['messages'][0]
        self.assertTrue(message['images'][0].startswith('iVBOR'))
        self.assertFalse(Backend.received[-1]['stream'])
    def test_plan_client(self):
        with patch.dict(os.environ,{'OLLAMA_TOKEN':self.token}):client=OllamaClient(self.url)
        target,_=client.plan('Find the red box'); self.assertEqual(target,'red')
    def test_reject_other_model(self):
        body=json.dumps({'model':'other','stream':False}).encode()
        with self.assertRaises(HTTPError) as ctx:self.request('/api/chat',{'Authorization':'Bearer '+self.token},body)
        self.assertEqual(ctx.exception.code,400)
    def test_bad_png_rejected_before_network(self):
        with self.assertRaises(ValueError):OllamaClient(self.url).recognize(b'not png')
