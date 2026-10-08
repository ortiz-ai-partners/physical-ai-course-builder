"""Audit pose episodes without changing recordings or starting training."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pose_task import CASES, FEATURE_NAMES, features


def inspect_episode(path):
    rows = [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
    if len(rows) < 3 or rows[0].get('type') != 'header' or rows[-1].get('type') != 'end':
        raise ValueError('Missing header, transitions, or end record')
    header, end = rows[0], rows[-1]
    transitions = rows[1:-1]
    if header.get('schema') != 'pose-demo-v1' or header.get('feature_names') != FEATURE_NAMES:
        raise ValueError('Unsupported pose recording format')
    case = CASES.get(header.get('case'))
    if case is None or header.get('goal') != case['target'] or header.get('split') != case['split']:
        raise ValueError('Case, goal, or split mismatch')
    if hashlib.sha256(header['scene_xml'].encode()).hexdigest() != header['scene_sha256']:
        raise ValueError('Scene hash mismatch')
    if end.get('frames') != len(transitions):
        raise ValueError('Frame count mismatch')
    previous = header['initial_state']
    hold = 0.0
    for index, row in enumerate(transitions):
        if row.get('type') != 'transition' or row.get('step') != index or row.get('goal') != header['goal']:
            raise ValueError('Unexpected frame or goal')
        state, nxt = row['state'], row['next_state']
        if not np.allclose(state['qpos'], previous['qpos'], atol=1e-9, rtol=0) or abs(state['time']-previous['time']) > 1e-8:
            raise ValueError('Discontinuous episode')
        if abs(nxt['time']-state['time']-.02) > 1e-8:
            raise ValueError('Invalid step timing')
        for key, observation in [('observation', state), ('next_observation', nxt)]:
            if not np.allclose(row[key], features(observation, header['goal']), atol=1e-9, rtol=0):
                raise ValueError('Goal observation does not match recorded state')
        action = np.asarray(row['action'])
        if action.shape != (3,) or not np.isfinite(action).all() or (np.abs(action) > 1).any() or action[2] != 0:
            raise ValueError('Invalid empty-vehicle action')
        f = row['next_observation']
        matched = np.hypot(f[0], f[1]) <= .08 and abs(np.degrees(np.arctan2(f[2], f[3]))) <= 8 and np.hypot(f[4], f[5]) < .03 and abs(f[6]) < .04
        hold = hold+.02 if matched else 0
        previous = nxt
    if end.get('success') and hold < 1-1e-8:
        raise ValueError('Success label lacks one second of matching observations')
    eligible = (header.get('source') == 'keyboard_teleoperation' and header['split'] == 'train' and end.get('success') is True)
    return {'file': Path(path).name, 'case': header['case'], 'split': header['split'],
            'source': header.get('source'), 'frames': len(transitions), 'success': end.get('success'),
            'eligible_for_training': eligible}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path, nargs='?', default=Path(__file__).resolve().parent/'recordings'/'pose')
    args = parser.parse_args()
    reports = []
    for path in sorted(args.folder.glob('*.jsonl')):
        try: reports.append(inspect_episode(path))
        except (ValueError, KeyError, TypeError) as exc:
            reports.append({'file': path.name, 'error': str(exc), 'eligible_for_training': False})
    print(json.dumps({'episodes': reports, 'training_started': False}, ensure_ascii=False, indent=2))
    return int(any('error' in r for r in reports))


if __name__ == '__main__': raise SystemExit(main())
