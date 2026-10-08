"""Validate editor targets and preview semantics without opening a window."""
import copy
import unittest
import tempfile
import threading
import urllib.request
import urllib.error
import json
import io
from unittest.mock import patch, Mock
from pathlib import Path
import designer
import numpy as np
from app import Simulation
from layout import validate_layout

EXAMPLE = {'schema': 1, 'blocks': [
    {'id': 'block_1', 'x': 1.5, 'y': 0.75, 'yaw': 0},
    {'id': 'block_2', 'x': 1.5, 'y': -0.75, 'yaw': 0}]}


class LayoutChecks(unittest.TestCase):
    def test_ramp_request_roundtrip_without_socket(self):
        """Exercise the real save/get/preview handler without network access."""
        value = copy.deepcopy(EXAMPLE)
        value['blocks'][0]['y'], value['blocks'][1]['y'] = 3, -3
        value['blocks'][0]['x'] = value['blocks'][1]['x'] = 4
        value['ramps'] = [{'id': 'ramp_1', 'x': 1.5, 'y': 0, 'height': 0.44}]
        (designer.ROOT / '.test-results').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=designer.ROOT / '.test-results') as folder:
            with patch.object(designer, 'ROOT', Path(folder)):
                h = designer.Handler.__new__(designer.Handler)
                h.server = Mock(server_port=12345, save_lock=threading.Lock(), preview_process=None)
                h.headers = {'Host': '127.0.0.1:12345', 'Origin': 'http://127.0.0.1:12345'}
                h.reply = Mock()
                def post(path):
                    payload = json.dumps(value).encode()
                    h.rfile = io.BytesIO(payload)
                    h.headers['Content-Length'] = str(len(payload))
                    h.path = path
                    h.do_POST()
                    return h.reply.call_args.args
                self.assertEqual(post('/api/save')[0], 200)
                h.path = '/api/layout'
                h.do_GET()
                self.assertEqual(h.reply.call_args.args, (200, validate_layout(value)))
                process = Mock()
                process.poll.return_value = None
                with patch('designer.subprocess.Popen', return_value=process) as launch:
                    self.assertEqual(post('/api/preview')[0], 200)
                    self.assertIn('--layout', launch.call_args.args[0])
                    self.assertEqual(post('/api/preview')[0], 409)
                    self.assertEqual(post('/api/build')[0], 400)
                    self.assertEqual(launch.call_count, 1)
                    h.server.preview_process = None
                    self.assertEqual(post('/api/build-ramp')[0], 200)
                    self.assertIn('--portable-ramp', launch.call_args.args[0])
                    self.assertEqual(launch.call_count, 2)
                    value['ramps'][0]['y'] = 0.25
                    self.assertEqual(post('/api/build-ramp')[0], 400)
                    self.assertEqual(launch.call_count, 2)

    def test_save_reload_and_reject_invalid(self):
        old_root = designer.ROOT
        (old_root / '.test-results').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=old_root / '.test-results') as folder:
            designer.ROOT = Path(folder)
            server = designer.make_server()
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            url = f'http://127.0.0.1:{server.server_port}'
            client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            def post(value, origin=url):
                req = urllib.request.Request(url + '/api/save', data=json.dumps(value).encode(),
                    headers={'Origin': origin, 'Content-Type': 'application/json'})
                return client.open(req, timeout=5)
            try:
                with post(EXAMPLE) as response:
                    self.assertEqual(response.status, 200)
                with client.open(url + '/api/layout', timeout=5) as response:
                    self.assertEqual(json.load(response), validate_layout(EXAMPLE))
                for data, origin, code in [({'schema': 1, 'blocks': []}, url, 400),
                                           (EXAMPLE, 'http://other.example', 403)]:
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        post(data, origin)
                    self.assertEqual(error.exception.code, code)
                self.assertEqual(len(list((designer.ROOT / 'designs').glob('layout_*.json'))), 1)
                process = Mock()
                process.poll.return_value = None
                with patch('designer.subprocess.Popen', return_value=process) as launch:
                    req = urllib.request.Request(url + '/api/build', data=json.dumps(EXAMPLE).encode(),
                        headers={'Origin': url, 'Content-Type': 'application/json'})
                    with client.open(req, timeout=5) as response:
                        self.assertTrue(json.load(response)['build'])
                    self.assertIn('--build', launch.call_args.args[0])
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        client.open(req, timeout=5)
                    self.assertEqual(error.exception.code, 409)
                    self.assertEqual(launch.call_count, 1)
            finally:
                server.shutdown()
                server.server_close()
                thread.join()
                designer.ROOT = old_root

    def test_invalid_designs(self):
        for change in ({'x': float('nan')}, {'x': 5}, {'x': 1.1}, {'yaw': 90},
                       {'x': True}, {'x': 1.5, 'y': -0.75}, {'id': 'block_2'}):
            layout = copy.deepcopy(EXAMPLE)
            layout['blocks'][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_layout(layout)

    def test_preview_positions_and_normal_scene(self):
        sim = Simulation(layout=EXAMPLE)
        state = sim.observe()
        for block in EXAMPLE['blocks']:
            np.testing.assert_allclose(state['objects'][block['id']]['position'][:2],
                                       [block['x'], block['y']], atol=1e-6)
        self.assertNotIn('block_3', state['objects'])
        self.assertIn('block_3', Simulation().observe()['objects'])

    def test_ramp_save_shape_and_preview(self):
        layout = copy.deepcopy(EXAMPLE)
        layout['blocks'][0]['y'], layout['blocks'][1]['y'] = 3, -3
        layout['ramps'] = [{'id': 'ramp_1', 'x': 1, 'y': 0.25, 'height': 0.4, 'yaw': 0}]
        saved = validate_layout(json.loads(json.dumps(validate_layout(layout))))
        self.assertEqual(saved['ramps'], layout['ramps'])
        sim = Simulation(layout=saved)
        top = sim.model.geom('portable_deck').id
        np.testing.assert_allclose(sim.data.body('portable_ramp').xpos, [1, 0.25, 0.08], atol=0.001)
        self.assertTrue(sim.model.geom_contype[top])
        for change in ({'height': 0.8}, {'x': float('nan')}, {'x': 5}, {'x': 0.1},
                       {'y': 3}, {'yaw': 90}, {'height': True}):
            bad = copy.deepcopy(saved)
            bad['ramps'][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_layout(bad)
        with self.assertRaises(ValueError):
            Simulation(layout=saved, construction=True)
        from ai_plan import load_plan, validate_plan
        plan = load_plan(designer.ROOT / 'examples' / 'ortiz-gate-plan.json')
        plan['layout'] = saved
        with self.assertRaises(ValueError):
            validate_plan(plan)


if __name__ == '__main__':
    unittest.main()
