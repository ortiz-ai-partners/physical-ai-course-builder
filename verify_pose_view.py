"""Check viewer control matches batch evaluation, including failed model inference."""
import json
import tempfile
from pathlib import Path
import numpy as np
from app import ROOT, Simulation
from pose_task import pose_xml, FEATURE_NAMES
from pose_view import PosePilot
from pose_evaluate import run_trial, PoseBaseline
from pose_bc import Policy


def compare(case, make_controller):
    expected = run_trial(case, make_controller())
    sim = Simulation(scene_xml=pose_xml(ROOT/'scene_fork_tracks.xml', case))
    pilot = PosePilot(sim, case, make_controller())
    while not pilot.done:
        action = pilot.action(sim)
        if not pilot.done: sim.step(*action)
    assert pilot.success == expected['success']
    assert abs(sim.data.time-expected['seconds']) < .001
    assert pilot.reason == expected['reason']
    for key, value in pilot.task.metrics.items():
        assert abs(value-expected['final_metrics'][key]) < 1e-9
    return {'case': case, 'success': pilot.success, 'seconds': expected['seconds']}


def main():
    results = [compare(case, lambda: PoseBaseline(True)) for case in ('back', 'rear-left', 'rotated-eval')]
    # Untrained constant-stop weights test inference plumbing, not learned skill.
    weights = {'w1': np.zeros((7,32)), 'b1': np.zeros(32),
               'w2': np.zeros((32,10)), 'b2': np.zeros(10)}
    weights['b2'][[2,7]] = 1
    policy = Policy(np.zeros(7), np.ones(7), weights)
    with tempfile.TemporaryDirectory(dir=ROOT/'.test-results', prefix='view-fixture-') as folder:
        path = Path(folder)/'untrained-test.npz'
        policy.save(path, {'schema': 'pose-bc-v1', 'feature_names': FEATURE_NAMES,
                           'training_source': 'untrained_test_fixture'})
        results.append(compare('back', lambda: Policy.load(path)[0]))
    assert not results[-1]['success']
    print(json.dumps({'viewer_matches_batch': results, 'human_training_performed': False}, indent=2))


if __name__ == '__main__': main()
