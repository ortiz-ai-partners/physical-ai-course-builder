"""Closed-loop pose evaluation: matched rule baselines and an optional learned policy."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from app import ROOT, Simulation
from pose_task import CASES, PoseTask, pose_xml, features, wrap
from pose_bc import Policy


class PoseBaseline:
    def __init__(self, reverse):
        self.reverse = reverse
        self.mode = 'move'
        self.direction = None
        self.speed = 0.0

    def action(self, observation):
        forward, left, sine, cosine = observation[:4]
        distance = math.hypot(forward, left)
        final_angle = math.atan2(sine, cosine)
        if self.mode == 'move' and distance < .04:
            self.mode = 'align'
        if self.mode == 'align' and abs(final_angle) < .06 and distance > .08:
            self.mode = 'move'; self.direction = None
        if self.mode == 'align':
            angle = final_angle
            target_speed = 0
        else:
            angle = math.atan2(left, forward)
            if self.direction is None:
                self.direction = -1 if self.reverse and abs(angle) > math.pi/2+.2 else 1
            if self.direction < 0: angle = wrap(angle+math.pi)
            target_speed = self.direction*min(.4, distance*.9) if abs(angle) < .2 else 0
        self.speed = float(np.clip(target_speed, self.speed-.025, self.speed+.025))
        turn = math.copysign(min(.9, max(.65, abs(angle)*2)), angle) if abs(angle) > .04 else 0
        return self.speed, turn, 0.0


def run_trial(case, controller, max_seconds=120):
    sim = Simulation(scene_xml=pose_xml(ROOT/'scene_fork_tracks.xml', case))
    task = PoseTask(sim, case)
    length = 0.0
    reverse_steps = 0
    previous = sim.data.body('dozer').xpos[:2].copy()
    while not task.done and sim.data.time < max_seconds:
        observation = features(sim.observe(), task.target)
        before = sim.data.qpos.copy()
        action = controller.action(observation)
        if len(action) != 3 or not np.isfinite(action).all() or max(abs(v) for v in action) > 1 or action[2] != 0:
            task.done = True; task.reason = 'Invalid policy action'; break
        assert np.array_equal(before, sim.data.qpos), 'Policy changed physical state directly'
        reverse_steps += int(action[0] < -.01)
        sim.step(*action); task.update(sim)
        position = sim.data.body('dozer').xpos[:2].copy()
        length += float(np.linalg.norm(position-previous)); previous = position
    if not task.done: task.reason = f'Evaluation time limit ({max_seconds}s)'
    return {'case': case, 'case_split': CASES[case]['split'], 'success': task.success,
            'reason': task.reason, 'seconds': round(sim.data.time, 2), 'distance_m': length,
            'reverse_steps': reverse_steps, 'final_metrics': task.metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=list(CASES), action='append')
    args = parser.parse_args()
    if args.output.exists(): parser.error('Choose a new evaluation output path')
    model, metadata = Policy.load(args.model) if args.model else (None, None)
    reports = []
    for case in args.case or list(CASES):
        for label, controller in [('forward_rules', PoseBaseline(False)), ('reverse_rules', PoseBaseline(True))]:
            reports.append(dict(run_trial(case, controller), controller=label))
        if model:
            reports.append(dict(run_trial(case, model), controller='behavior_cloning'))
    result = {'schema': 'pose-evaluation-v1', 'trials': reports,
              'model_training_source': metadata.get('training_source') if metadata else None,
              'learned_policy_evaluated': model is not None,
              'note': 'Same initial scenes and pose criteria for each controller. Rules here add final heading alignment; older position-only timings are not directly comparable.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
