"""Audit pose episodes without changing recordings or starting training."""
import argparse
from collections import Counter
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
    parser.add_argument('--summary', action='store_true', help='Show collection counts and next steps in Japanese.')
    args = parser.parse_args()
    if args.summary:
        summary = collection_summary(args.folder)
        names = {'back': '後退', 'front': '前進', 'rear-left': '左斜め後ろ'}
        print('お手本の収集状況（元の記録は変更しません）\n')
        for case, label in names.items():
            count = summary['unique_successes'][case]
            remaining = max(0, 2-count)
            optional = '・後からでも可' if case == 'rear-left' and count == 0 else ''
            print(f'{label}: 有効な成功 {count} 本 / 最初の目安 2 本 / あと {remaining} 本{optional}')
        print(f'\n重複: {summary["duplicates"]} 本 / 対象外: {summary["excluded"]} 本 / 要点検: {len(summary["issues"])} 本')
        for issue in summary['issues']:
            print(f'  {issue["file"]}: {issue["reason"]}')
        print('\n少数データでの学習試行を始める準備ができています。' if summary['ready_for_first_trial'] else '\nまず前進・後退を各2本。1本だけ成功したケースは、もう1本追加してください。')
        print('これは分割の最低目安です。学習性能を保証する本数ではありません。学習は自動で始まりません。')
        return 0
    reports = []
    for path in sorted(args.folder.glob('*.jsonl')):
        try: reports.append(inspect_episode(path))
        except (ValueError, KeyError, TypeError) as exc:
            reports.append({'file': path.name, 'error': str(exc), 'eligible_for_training': False})
    print(json.dumps({'episodes': reports, 'training_started': False}, ensure_ascii=False, indent=2))
    return int(any('error' in r for r in reports))


def collection_summary(folder):
    from pose_prepare import encode_actions
    counts = Counter({'back': 0, 'front': 0, 'rear-left': 0})
    seen, issues = set(), []
    duplicates = excluded = 0
    for path in sorted(Path(folder).glob('*.jsonl')):
        try:
            raw = path.read_bytes()
            report = inspect_episode(path)
            if not report['eligible_for_training']:
                excluded += 1; continue
            digest = hashlib.sha256(raw).hexdigest()
            if digest in seen:
                duplicates += 1; continue
            rows = [json.loads(line) for line in raw.decode('utf-8').splitlines() if line.strip()]
            encode_actions(np.asarray([r['action'] for r in rows[1:-1]]))
            seen.add(digest); counts[report['case']] += 1
        except (ValueError, KeyError, TypeError, OSError) as exc:
            issues.append({'file': path.name, 'reason': str(exc)})
    ready = counts['back'] >= 2 and counts['front'] >= 2 and all(n == 0 or n >= 2 for n in counts.values())
    return {'unique_successes': dict(counts), 'duplicates': duplicates,
            'excluded': excluded, 'issues': issues, 'ready_for_first_trial': ready,
            'training_started': False}


if __name__ == '__main__': raise SystemExit(main())
