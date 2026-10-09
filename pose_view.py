"""Watch pose rules or a saved policy using the same task as batch evaluation."""
import argparse
from pathlib import Path
import numpy as np
from pose_task import CASES, PoseTask, features


class PosePilot:
    def __init__(self, sim, case, controller):
        self.task = PoseTask(sim, case)
        self.controller = controller

    @property
    def done(self): return self.task.done

    @property
    def success(self): return self.task.success

    @property
    def reason(self): return self.task.reason

    @property
    def stage(self):
        return ('SUCCESS' if self.success else 'FAILED') if self.done else 'POSE CONTROL'

    def action(self, sim):
        if sim.data.time > self.task.last_time:
            self.task.update(sim)
        if self.done: return (0, 0, 0)
        before = sim.data.qpos.copy()
        action = self.controller.action(features(sim.observe(), self.task.target))
        assert np.array_equal(before, sim.data.qpos), 'Policy changed physical state directly'
        if len(action) != 3 or not np.isfinite(action).all() or max(abs(v) for v in action) > 1 or action[2] != 0:
            self.task.done = True
            self.task.reason = 'Invalid policy action'
            return (0, 0, 0)
        return action


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--model', type=Path)
    group.add_argument('--rules', choices=('forward', 'reverse'))
    parser.add_argument('--case', choices=list(CASES), default='back')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    from pose_evaluate import PoseBaseline
    from pose_bc import Policy
    from app import run
    if args.model:
        controller, metadata = Policy.load(args.model)
        label = 'LOCAL MODEL INFERENCE / NO TRAINING\nSource: '+str(metadata.get('training_source', 'unknown'))
    else:
        controller = PoseBaseline(args.rules == 'reverse')
        label = args.rules.upper()+' RULES / NO MODEL / NO TRAINING'
    return run(screenshot=args.screenshot, pose_view=(args.case, controller, label))


if __name__ == '__main__': raise SystemExit(main())
