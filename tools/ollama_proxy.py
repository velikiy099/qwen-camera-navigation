#!/usr/bin/env python3
"""Small authenticated HOST-side proxy; keep Ollama itself bound to localhost.

Only /api/tags and non-streaming /api/chat for the configured model are allowed.
No arbitrary URLs, model downloads, file browsing, CORS or command execution.
Use only on a trusted local network; the proxy transport is HTTP, not TLS.
"""
from __future__ import annotations
import hmac
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
from urllib.parse import urlsplit

MAX_BODY = 8 * 1024 * 1024

def make_handler(token: str, backend='http://127.0.0.1:11434', model='qwen2.5vl:3b'):
    opener = build_opener(ProxyHandler({}))
    class Handler(BaseHTTPRequestHandler):
        server_version = 'PAILocalProxy/1'
        def log_message(self, fmt, *args):
            # Log path/status only, never auth headers or image/prompt content.
            print('proxy:', fmt % args, flush=True)
        def respond(self, status, body, content_type='application/json'):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        def forward(self):
            self.connection.settimeout(150)
            auth = self.headers.get('Authorization', '')
            if not hmac.compare_digest(auth, 'Bearer ' + token):
                self.respond(401, b'{"error":"authentication required"}'); return
            if self.headers.get('Origin'):
                self.respond(403, b'{"error":"browser requests are not allowed"}'); return
            path = urlsplit(self.path)
            if path.query or path.fragment or path.scheme or path.netloc:
                self.respond(400, b'{"error":"invalid path"}'); return
            if (self.command, path.path) not in (('GET', '/api/tags'), ('POST', '/api/chat')):
                self.respond(403, b'{"error":"endpoint not allowed"}'); return
            data = None
            if self.command == 'POST':
                try:
                    size = int(self.headers.get('Content-Length', '-1'))
                    if not 0 < size <= MAX_BODY:
                        raise ValueError('Request too large or missing length')
                    data = self.rfile.read(size)
                    if len(data) != size: raise ValueError('Truncated request')
                    payload = json.loads(data)
                    if payload.get('model') != model or payload.get('stream') is not False:
                        raise ValueError('Only the configured local model and stream=false are allowed')
                except (ValueError, TypeError, AttributeError) as exc:
                    self.respond(400, json.dumps({'error': str(exc)}).encode()); return
            req = Request(backend + path.path, data=data, method=self.command,
                          headers={'Content-Type': 'application/json'})
            try:
                with opener.open(req, timeout=140) as upstream:
                    body = upstream.read(2 * 1024 * 1024)
                    self.respond(upstream.status, body)
            except HTTPError as exc:
                self.respond(exc.code, exc.read(4096))
            except Exception:
                self.respond(502, b'{"error":"local Ollama unavailable or timed out"}')
        def do_GET(self): self.forward()
        def do_POST(self): self.forward()
    return Handler

def main():
    token = os.environ.get('OLLAMA_TOKEN', '')
    if len(token) < 24:
        raise SystemExit('Missing/short OLLAMA_TOKEN. Start with ./run.sh proxy after ./run.sh init.')
    bind = os.environ.get('PAI_PROXY_BIND', '0.0.0.0')
    port = int(os.environ.get('PAI_PROXY_PORT', '11435'))
    server = ThreadingHTTPServer((bind, port), make_handler(token, model=os.environ.get('OLLAMA_MODEL', 'qwen2.5vl:3b')))
    print(f'Authenticated local proxy listening on {bind}:{port}; forwarding only to 127.0.0.1:11434.')
    print('Keep this terminal open. Ctrl+C stops the proxy. Use a trusted local network; HTTP is not encrypted.')
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()
if __name__ == '__main__': main()
