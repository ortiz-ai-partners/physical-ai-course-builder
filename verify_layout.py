"""Validate editor targets and preview semantics without opening a window."""
import copy
import unittest
import tempfile
import threading
import urllib.request
import urllib.error
import json
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


if __name__ == '__main__':
    unittest.main()
