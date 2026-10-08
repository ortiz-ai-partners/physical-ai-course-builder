"""Local-only course editor. No AI, network API, or extra dependencies."""
import argparse
import json
import subprocess
import sys
import threading
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from layout import validate_layout

ROOT = Path(__file__).resolve().parent


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, value, kind='application/json; charset=utf-8'):
        data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/':
            return self.reply(200, (ROOT / 'editor.html').read_bytes(), 'text/html; charset=utf-8')
        if self.path == '/api/layout':
            path = ROOT / 'designs' / 'latest.json'
            if path.exists():
                return self.reply(200, json.loads(path.read_text(encoding='utf-8')))
            return self.reply(200, {'schema': 1, 'blocks': [
                {'id': 'block_1', 'x': 1.5, 'y': 0.75, 'yaw': 0},
                {'id': 'block_2', 'x': 1.5, 'y': -0.75, 'yaw': 0}]})
        self.reply(404, {'error': '見つかりません。'})

    def do_POST(self):
        # Only the page served by this process may issue browser writes.
        expected = f'http://127.0.0.1:{self.server.server_port}'
        if self.headers.get('Origin') != expected or self.headers.get('Host') != expected[7:]:
            return self.reply(403, {'error': 'この設計画面から操作してください。'})
        if self.path not in ('/api/save', '/api/preview'):
            return self.reply(404, {'error': '見つかりません。'})
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size < 16384:
                raise ValueError('配置データのサイズが違います。')
            layout = validate_layout(json.loads(self.rfile.read(size)))
            with self.server.save_lock:
                folder = ROOT / 'designs'
                folder.mkdir(exist_ok=True)
                path = folder / ('layout_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
                content = json.dumps(layout, ensure_ascii=False, indent=2) + '\n'
                path.write_text(content, encoding='utf-8')
                temporary = folder / 'latest.tmp'
                temporary.write_text(content, encoding='utf-8')
                temporary.replace(folder / 'latest.json')
                if self.path == '/api/preview':
                    previous = self.server.preview_process
                    if previous is not None and previous.poll() is None:
                        return self.reply(409, {'error': '配置は保存しました。前の3Dプレビューを閉じてから、もう一度開いてください。'})
                    # This is an explicitly requested visible interactive window.
                    self.server.preview_process = subprocess.Popen(
                        [sys.executable, str(ROOT / 'app.py'), '--layout', str(path)], cwd=ROOT)
            self.reply(200, {'saved': path.name, 'preview': self.path == '/api/preview'})
        except (ValueError, TypeError, OSError) as error:
            self.reply(400, {'error': str(error)})


def make_server(port=0):
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.save_lock = threading.Lock()
    server.preview_process = None
    return server


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    server = make_server(args.port)
    url = f'http://127.0.0.1:{server.server_port}'
    print(url, flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
