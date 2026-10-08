"""Synthetic-only tests of episode splits, model gradients, CPU training and evaluation."""
import json
import tempfile
from pathlib import Path
import numpy as np
from app import ROOT, Simulation
from pose_task import PoseTask, PoseRecorder, pose_xml, FEATURE_NAMES
from pose_prepare import prepare, load_dataset
from pose_bc import Policy, train_arrays, loss_and_gradient
from pose_evaluate import run_trial, PoseBaseline


def fixture(folder, case, idle):
    xml = pose_xml(ROOT/'scene_fork_tracks.xml', case)
    sim = Simulation(scene_xml=xml); task = PoseTask(sim, case)
    # Simulate the human recorder protocol solely in a disposable test folder.
    # These are not actual human demonstrations and are never put in recordings/.
    rec = PoseRecorder(task, xml, folder, source='keyboard_teleoperation')
    rec.start(sim)
    for step in range(1000):
        before = sim.observe()
        error = task.target[0]-before['objects']['dozer']['position'][0]
        action = [float(np.sign(error))*.3 if abs(error) > .03 and step >= idle else 0, 0, 0]
        controls = sim.step(*action); task.update(sim)
        rec.write({'type':'transition', 'step':rec.frames, 'state':before, 'action':action,
                   'actuator_command':controls.tolist(), 'next_state':sim.observe()})
        rec.frames += 1
        if task.done: break
    assert task.success, task.metrics
    rec.stop('task_success')


def main():
    test_root = ROOT/'.test-results'; test_root.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=test_root, prefix='learning-fixtures-') as temp:
        folder = Path(temp)
        try: prepare(folder, folder/'empty.npz'); raise AssertionError('Empty data trained')
        except ValueError: pass
        for case in ('back', 'front'):
            for idle in (5, 15): fixture(folder, case, idle)
        original = next(folder.glob('*.jsonl'))
        (folder/'duplicate.jsonl').write_bytes(original.read_bytes())
        manifest = prepare(folder, folder/'dataset.npz')
        arrays, loaded = load_dataset(folder/'dataset.npz')
        assert len(manifest['episodes']) == 4
        assert len(manifest['skipped']) == 1 and manifest['skipped'][0]['reason'] == 'Duplicate recording'
        assert not set(arrays['train_episode']) & set(arrays['val_episode'])
        for case in ('back', 'front'):
            assert {e['split'] for e in manifest['episodes'] if e['case'] == case} == {'train', 'val'}
        try: prepare(folder, folder/'dataset.npz'); raise AssertionError('Existing data overwritten')
        except FileExistsError: pass
        corrupted = dict(arrays)
        corrupted['val_episode'] = arrays['val_episode'].copy()
        corrupted['val_episode'][0] = arrays['train_episode'][0]
        np.savez_compressed(folder/'leaked.npz', **corrupted, metadata=np.array(json.dumps(loaded)))
        try: load_dataset(folder/'leaked.npz'); raise AssertionError('Leaked episode accepted')
        except ValueError: pass
        # Numeric toy task tests optimization; it is not robot learning evidence.
        rng = np.random.default_rng(31)
        x = rng.normal(size=(1000,7))
        y = np.stack([np.where(x[:,0] > 0, 4, 0), np.where(x[:,1] > 0, 3, 1)], axis=1)
        model, report = train_arrays(x[:800], y[:800], x[800:], y[800:], epochs=80)
        assert report['best_validation_loss'] < report['initial_validation_loss']*.2
        assert report['validation_joint_action_accuracy'] > .9
        # Finite differences catch incorrect backprop derivatives.
        _, grads = loss_and_gradient(model, x[:8], y[:8])
        old = model.weights['w1'][0,0]; epsilon = 1e-5
        model.weights['w1'][0,0] = old+epsilon; upper = loss_and_gradient(model, x[:8], y[:8])[0]
        model.weights['w1'][0,0] = old-epsilon; lower = loss_and_gradient(model, x[:8], y[:8])[0]
        model.weights['w1'][0,0] = old
        assert abs((upper-lower)/(2*epsilon)-grads['w1'][0,0]) < 1e-5
        model.save(folder/'toy.npz', {'schema':'pose-bc-v1','feature_names':FEATURE_NAMES,'training_source':'synthetic_test'})
        restored, meta = Policy.load(folder/'toy.npz')
        assert np.allclose(model.probabilities(x[:10]), restored.probabilities(x[:10]))
        synthetic_rollout = run_trial('back', restored, max_seconds=2)
        assert not synthetic_rollout['success']  # No useful robot skill claimed.
        baseline = run_trial('back', PoseBaseline(True))
        assert baseline['success']
        summary = {'episode_leakage_rejected': True, 'overwrite_rejected': True,
                   'gradient_check': 'PASS', 'toy_validation_action_accuracy': report['validation_joint_action_accuracy'],
                   'policy_save_load': 'PASS', 'reverse_pose_baseline': baseline,
                   'synthetic_policy_short_rollout': synthetic_rollout,
                   'human_robot_model_trained': False,
                   'note':'All generated records and weights are disposable synthetic test fixtures.'}
        (test_root/'pose-learning-verification.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(json.dumps(summary,indent=2))


if __name__ == '__main__': main()
