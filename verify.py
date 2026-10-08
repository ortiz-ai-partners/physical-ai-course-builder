"""Contact, control, release and recording checks; no learning or GPU job."""
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import app


def yaw(sim):
    w, x, y, z = sim.data.body('dozer').xquat
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def main():
    sim = app.Simulation('blade')
    results = {}
    start = sim.data.body('dozer').xpos.copy()
    block = sim.data.body('block_1').xpos.copy()
    for _ in range(150):
        sim.step(1, 0, 0)
    results['forward_m'] = float(sim.data.body('dozer').xpos[0] - start[0])
    results['pushed_block_m'] = float(sim.data.body('block_1').xpos[0] - block[0])
    assert results['forward_m'] > 1.0
    assert results['pushed_block_m'] > 0.5
    for _ in range(50):
        sim.step()
    results['released_speed_m_s'] = float(np.linalg.norm(sim.data.qvel[:2]))
    assert results['released_speed_m_s'] < 0.05
    sim.reset()
    start = sim.data.body('dozer').xpos.copy()
    for _ in range(100):
        sim.step(-1, 0, 0)
    results['reverse_m'] = float(sim.data.body('dozer').xpos[0] - start[0])
    assert results['reverse_m'] < -0.5
    for sign, label in [(1, 'left'), (-1, 'right')]:
        sim.reset()
        for _ in range(150):
            sim.step(0, sign, 0)
        results[label + '_yaw_rad'] = yaw(sim)
        assert yaw(sim) * sign > 0.3
    sim.reset()
    for _ in range(100):
        sim.step(0, 0, 1)
    results['blade_high_m'] = float(sim.data.joint('lift').qpos[0])
    assert 0.45 < results['blade_high_m'] < 0.52
    for _ in range(100):
        sim.step(0, 0, -1)
    results['blade_low_m'] = float(sim.data.joint('lift').qpos[0])
    assert abs(results['blade_low_m']) < 0.02
    assert np.isfinite(sim.data.qpos).all()

    # Record via the same Recorder class as the interactive app, outside user recordings.
    original_root = app.ROOT
    folder = original_root / '.test-results' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    if folder.is_dir():
        (folder / 'scene.xml').write_bytes((original_root / 'scene.xml').read_bytes())
        app.ROOT = folder
        try:
            recorder = app.Recorder()
            recorder.start(sim)
            for i in range(10):
                state = sim.observe()
                command = sim.step(1, 0, 0)
                recorder.write({'type': 'transition', 'step': i, 'state': state,
                                'action': [1, 0, 0], 'actuator_command': command.tolist(),
                                'next_state': sim.observe()})
                recorder.frames += 1
                # Rendering recomputes derived poses: it must not change observations.
                before_render = sim.observe()
                sim.update_tracks()
                assert sim.observe() == before_render
            recorder.stop()
            rows = [json.loads(line) for line in recorder.path.read_text(encoding='utf-8').splitlines()]
            assert len(rows) == 12 and rows[-1]['frames'] == 10
            assert all(abs(row['next_state']['time'] - row['state']['time'] - app.CONTROL_DT) < 1e-8
                       for row in rows[1:-1])
            assert rows[1]['state']['qpos'] != rows[-2]['state']['qpos']
            assert all(a['next_state'] == b['state'] for a, b in zip(rows[1:-2], rows[2:-1]))
            results['recording_transitions_checked'] = 10
        finally:
            app.ROOT = original_root
    # Lift, transport and release must follow physical fork contact, not a weld.
    fork = app.Simulation('fork')
    original_box = fork.data.body('block_1').xpos.copy()
    for _ in range(95):
        fork.step(0.3, 0, 0)
    assert np.linalg.norm(fork.data.body('block_1').xpos[:2] - original_box[:2]) < 0.02
    for _ in range(100):
        fork.step(0, 0, 1)
    results['fork_box_lift_m'] = float(fork.data.body('block_1').xpos[2] - original_box[2])
    assert results['fork_box_lift_m'] > 0.4
    for _ in range(100):
        fork.step(-0.3, 0, 0)
    results['fork_box_transport_m'] = float(original_box[0] - fork.data.body('block_1').xpos[0])
    assert results['fork_box_transport_m'] > 0.4
    assert fork.data.body('block_1').xpos[2] > original_box[2] + 0.35
    for _ in range(100):
        fork.step(0, 0, -1)
    deposited = fork.data.body('block_1').xpos.copy()
    for _ in range(120):
        fork.step(-0.3, 0, 0)
    assert abs(fork.data.body('block_1').xpos[2] - original_box[2]) < 0.02
    assert np.linalg.norm(fork.data.body('block_1').xpos - deposited) < 0.03
    assert fork.model.neq == 0
    assert np.isfinite(fork.data.qpos).all()
    results['fork_release'] = 'PASS'
    results['status'] = 'PASS'
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
