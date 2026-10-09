"""Replay recorded actuator choices through physics; never teleport between frames."""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from pose_dataset import inspect_episode


def load_episode(path):
    inspect_episode(path)
    rows = [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
    if rows[0]['engine_version'] != mujoco.__version__:
        raise ValueError('Replay requires the recorded MuJoCo version')
    if type(rows[-1].get('success')) is not bool:
        raise ValueError('Recording must have an explicit success/failure label')
    return rows


class PoseReplay:
    def __init__(self, sim, rows):
        self.header, self.frames, self.end = rows[0], rows[1:-1], rows[-1]
        self.index = 0
        self.done = self.success = False
        self.stage, self.reason = 'REPLAY', ''
        self.max_position_error = self.max_velocity_error = 0.0
        initial = self.header['initial_state']
        # One initial restoration. No state restoration occurs during replay.
        if 'integration_state' in self.header:
            state_type = mujoco.mjtState.mjSTATE_INTEGRATION
            vector = np.asarray(self.header['integration_state'], dtype=float)
            if vector.shape != (mujoco.mj_stateSize(sim.model, state_type),) or not np.isfinite(vector).all():
                raise ValueError('Invalid integration state')
            mujoco.mj_setState(sim.model, sim.data, vector, state_type)
            self.restore_mode = 'full_integration_state'
        else:
            for name in ('qpos', 'qvel'):
                vector = np.asarray(initial[name], dtype=float)
                if vector.shape != getattr(sim.data, name).shape or not np.isfinite(vector).all():
                    raise ValueError('Invalid legacy initial state')
                getattr(sim.data, name)[:] = vector
            sim.data.time = initial['time']
            self.restore_mode = 'legacy_partial_state'
        sim.lift_target = float(initial['lift_target'])
        sim.observe()

    def action(self, sim):
        if self.done: return (0, 0, 0)
        expected = self.frames[self.index]['state'] if self.index < len(self.frames) else self.frames[-1]['next_state']
        qerror = float(np.max(np.abs(sim.data.qpos-np.asarray(expected['qpos']))))
        verror = float(np.max(np.abs(sim.data.qvel-np.asarray(expected['qvel']))))
        self.max_position_error = max(self.max_position_error, qerror)
        self.max_velocity_error = max(self.max_velocity_error, verror)
        if not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all() or qerror > 1e-4 or verror > 1e-3 or abs(sim.data.time-expected['time']) > 1e-8:
            self.done = True; self.stage = 'STOPPED'
            self.reason = f'Replay diverged at frame {self.index}'
            return (0, 0, 0)
        if self.index == len(self.frames):
            self.done = self.success = True; self.stage = 'COMPLETE'
            self.reason = 'Replay verified; recorded task '+('succeeded' if self.end['success'] else 'did not succeed')
            return (0, 0, 0)
        action = self.frames[self.index]['action']
        self.index += 1
        self.stage = f'REPLAY {self.index}/{len(self.frames)}'
        return action

    def report(self):
        return {'replay_verified': self.success, 'demonstration_success': self.end['success'],
                'frames_replayed': self.index, 'reason': self.reason, 'restore_mode': self.restore_mode,
                'max_qpos_error': self.max_position_error, 'max_qvel_error': self.max_velocity_error,
                'new_ai_inference': False, 'training_performed': False}


def evaluate(path):
    from app import Simulation
    rows = load_episode(path)
    sim = Simulation(scene_xml=rows[0]['scene_xml'])
    replay = PoseReplay(sim, rows)
    while not replay.done:
        action = replay.action(sim)
        if not replay.done: sim.step(*action)
    return replay.report()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path, nargs='?')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--view', action='store_true', help='View specified recording, or newest recording if omitted.')
    args = parser.parse_args()
    if args.recording is None:
        if not args.view: parser.error('A recording is required for report generation')
        files = list((Path(__file__).resolve().parent/'recordings/pose').glob('*.jsonl'))
        if not files: parser.exit(1, 'No pose recordings yet. Record a demonstration first.\n')
        args.recording = max(files, key=lambda p: p.stat().st_mtime_ns)
    if args.view:
        from app import run
        try: rows = load_episode(args.recording)
        except (ValueError, KeyError, TypeError, OSError) as exc: parser.exit(1, str(exc)+'\n')
        print('Replaying: '+args.recording.name)
        raise SystemExit(run(replay_rows=rows))
    if args.report is None: parser.error('--report is required unless --view is used')
    if args.report.exists(): parser.error('Choose a new report path; recordings are never overwritten')
    result = evaluate(args.recording)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['replay_verified'] else 1)
