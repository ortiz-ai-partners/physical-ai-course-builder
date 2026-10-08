"""Export audited human episodes; split whole demonstrations, never adjacent frames."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pose_dataset import inspect_episode
from pose_task import FEATURE_NAMES, CASES

FORWARD_LEVELS = np.array([-1., -.3, 0., .3, 1.])
TURN_LEVELS = np.array([-1., -.65, 0., .65, 1.])


def encode_actions(actions):
    labels = []
    for axis, levels in enumerate((FORWARD_LEVELS, TURN_LEVELS)):
        distances = np.abs(actions[:, axis, None]-levels)
        if np.max(np.min(distances, axis=1)) > 1e-6:
            raise ValueError('Unexpected keyboard action level; inspect before training')
        labels.append(np.argmin(distances, axis=1))
    return np.stack(labels, axis=1)


def prepare(folder, output, seed=1729):
    output = Path(output)
    if output.exists(): raise FileExistsError('Choose a new dataset output path')
    groups, skipped, seen = {}, [], set()
    for path in sorted(Path(folder).glob('*.jsonl')):
        try:
            raw = path.read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            report = inspect_episode(path)
            if not report['eligible_for_training']:
                skipped.append({'file': path.name, 'reason': 'Not a successful human train episode'})
                continue
            if digest in seen:
                skipped.append({'file': path.name, 'reason': 'Duplicate recording'})
                continue
            seen.add(digest)
            rows = [json.loads(line) for line in raw.decode('utf-8').splitlines() if line.strip()]
            transitions = rows[1:-1]
            x = np.asarray([r['observation'] for r in transitions], dtype=np.float64)
            y = encode_actions(np.asarray([r['action'] for r in transitions]))
            groups.setdefault(report['case'], []).append((path.name, digest, x, y))
        except (ValueError, KeyError, TypeError) as exc:
            skipped.append({'file': path.name, 'reason': str(exc)})
    if not groups:
        raise ValueError('No eligible human episodes. Record successful demonstrations first; test files are excluded.')
    short = {case: len(items) for case, items in groups.items() if len(items) < 2}
    if short:
        raise ValueError(f'Need at least two different successful episodes per collected case: {short}')
    arrays = {key: [] for key in ('train_x', 'train_y', 'train_episode', 'val_x', 'val_y', 'val_episode')}
    manifest = {'schema': 'pose-dataset-v1', 'dataset_kind': 'human_pose_demos',
                'feature_names': FEATURE_NAMES, 'split_seed': seed, 'episodes': [], 'skipped': skipped,
                'note': 'Validation holds out whole human demonstrations. Evaluation cases are excluded entirely.'}
    eid = 0
    for case, episodes in sorted(groups.items()):
        # Stable content-based ordering; filenames/timestamps do not decide split.
        episodes.sort(key=lambda e: hashlib.sha256(f'{seed}:{e[1]}'.encode()).hexdigest())
        validation_count = max(1, round(len(episodes)*.2))
        for index, (name, digest, x, y) in enumerate(episodes):
            split = 'val' if index < validation_count else 'train'
            arrays[split+'_x'].append(x); arrays[split+'_y'].append(y)
            arrays[split+'_episode'].append(np.full(len(x), eid, dtype=np.int64))
            manifest['episodes'].append({'id': eid, 'file': name, 'sha256': digest,
                                         'case': case, 'split': split, 'frames': len(x)})
            eid += 1
    merged = {key: np.concatenate(values) for key, values in arrays.items()}
    output.parent.mkdir(parents=True, exist_ok=True)
    # Explicit handle prevents numpy from silently appending another extension.
    with output.open('xb') as handle:
        np.savez_compressed(handle, **merged, metadata=np.array(json.dumps(manifest, ensure_ascii=False)))
    return manifest


def load_dataset(path):
    with np.load(path, allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in ('train_x', 'train_y', 'train_episode', 'val_x', 'val_y', 'val_episode')}
        manifest = json.loads(str(data['metadata']))
    if manifest.get('schema') != 'pose-dataset-v1' or manifest.get('dataset_kind') != 'human_pose_demos' or manifest.get('feature_names') != FEATURE_NAMES:
        raise ValueError('Not a supported human pose dataset')
    if set(arrays['train_episode']) & set(arrays['val_episode']):
        raise ValueError('Episode leakage between train and validation')
    for split in ('train', 'val'):
        x, y, episode = [arrays[split+'_'+k] for k in ('x', 'y', 'episode')]
        if x.ndim != 2 or x.shape[1] != 7 or not len(x) or not np.isfinite(x).all():
            raise ValueError('Invalid feature matrix')
        if y.shape != (len(x), 2) or not np.issubdtype(y.dtype, np.integer) or (y < 0).any() or (y > 4).any():
            raise ValueError('Invalid action labels')
        if episode.shape != (len(x),): raise ValueError('Invalid episode IDs')
        entries = [entry for entry in manifest['episodes'] if entry['split'] == split]
        if set(episode) != {entry['id'] for entry in entries}:
            raise ValueError('Episode manifest mismatch')
        for entry in entries:
            if CASES.get(entry['case'], {}).get('split') != 'train':
                raise ValueError('Evaluation case leaked into fitting data')
            if np.count_nonzero(episode == entry['id']) != entry['frames']:
                raise ValueError('Manifest frame count mismatch')
    return arrays, manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, default=Path(__file__).resolve().parent/'recordings/pose')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = prepare(args.folder, args.output)
    except (ValueError, FileExistsError) as exc:
        parser.exit(1, str(exc)+'\n')
    print(json.dumps({'episodes': result['episodes'], 'skipped': result['skipped'], 'training_started': False}, ensure_ascii=False, indent=2))
