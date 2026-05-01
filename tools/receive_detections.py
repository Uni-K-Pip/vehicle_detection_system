#!/usr/bin/env python3
# Copyright 2026 kohei
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Minimal HTTP receiver for detection_sender_node payloads.

Usage:
    python3 tools/receive_detections.py --host 0.0.0.0 --port 8080

The endpoint accepts POST requests with JSON bodies in the schema produced
by detection_sender_node. Each payload is logged to stdout and a 204 No
Content response is returned. Useful when verifying the HTTP path of the
sender during development.
"""

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class DetectionHandler(BaseHTTPRequestHandler):
    def do_POST(self):  # noqa: N802 - http.server API
        length = int(self.headers.get('Content-Length', '0') or '0')
        body = self.rfile.read(length) if length > 0 else b''
        try:
            payload = json.loads(body.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(f'invalid JSON: {exc}'.encode('utf-8'))
            return
        timestamp = payload.get('timestamp', '<no-timestamp>')
        frame_id = payload.get('frame_id', '<no-frame>')
        detections = payload.get('detections', [])
        print(
            f'[{timestamp}] frame={frame_id} detections={len(detections)}',
            flush=True,
        )
        for det in detections:
            print(f'  {json.dumps(det)}', flush=True)
        self.send_response(204)
        self.end_headers()

    def log_message(self, fmt, *args):  # noqa: N802 - http.server API
        # Silence default per-request logging; we print a richer summary above.
        return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args(argv)

    server = ThreadingHTTPServer((args.host, args.port), DetectionHandler)
    print(
        f'Listening on http://{args.host}:{args.port}/ for detection POSTs',
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('shutting down', flush=True)
    finally:
        server.server_close()


if __name__ == '__main__':
    sys.exit(main())
