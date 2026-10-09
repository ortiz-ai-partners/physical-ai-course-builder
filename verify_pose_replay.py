"""Physical replay, moving initial state, divergence, and failed-demo distinction."""
import json
import tempfile
from pathlib import Path
import numpy as np
from app import ROOT, Simulation
from pose_task import PoseTask, PoseRecorder, pose_xml
from pose_replay import evaluate, load_episode, PoseReplay


def record(folder, stop_early=False):
    xml = pose_xml(ROOT/'scene_fork_tracks.xml', 'back')
    sim = Simulation(scene_xml=xml)
    task = PoseTask(sim, 'back')
    # Start recording with nonzero velocity, as when F is pressed mid-motion.
    for _ in range(10): sim.step(-.3); task.update(sim)
    recorder = PoseRecorder(task, xml, folder, source='scripted_test')
    recorder.start(sim)
    for _ in range(5 if stop_early else 1000):
        before = sim.observe()
        action = [-.3 if before['objects']['dozer']['position'][0] > -.97 else 0, 0, 0]
        controls = sim.step(*action); task.update(sim)
        recorder.write({'type':'transition','step':recorder.frames,'state':before,'action':action,
                        'actuator_command':controls.tolist(),'next_state':sim.observe()})
        recorder.frames += 1
        if task.done: break
    recorder.stop('user' if stop_early else 'task_success')
    assert task.success != stop_early
    return recorder.path


def main():
    base = ROOT/'.test-results'; base.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=base, prefix='replay-fixture-') as tmp:
        folder = Path(tmp)
        success_path = record(folder)
        report = evaluate(success_path)
        assert report['replay_verified'] and report['demonstration_success'], report
        assert report['restore_mode'] == 'full_integration_state'
        failed = evaluate(record(folder, stop_early=True))
        assert failed['replay_verified'] and not failed['demonstration_success']
        rows = load_episode(success_path)
        sim = Simulation(scene_xml=rows[0]['scene_xml']); pilot = PoseReplay(sim, rows)
        while not pilot.done:
            before = sim.data.qpos.copy(); action = pilot.action(sim)
            assert np.array_equal(before, sim.data.qpos)
            if not pilot.done: sim.step(*action)
        tampered = json.loads(success_path.read_text(encoding='utf-8').splitlines()[1])
        tampered['action'][0] = 1
        lines = success_path.read_text(encoding='utf-8').splitlines()
        lines[1] = json.dumps(tampered)
        corrupt = folder/'changed-action.jsonl'; corrupt.write_text('\n'.join(lines), encoding='utf-8')
        divergence = evaluate(corrupt)
        assert not divergence['replay_verified'] and 'diverged' in divergence['reason']
        result = {'successful_demo': report, 'failed_demo_replays': failed,
                  'changed_action_rejected': divergence, 'no_per_frame_teleport': True,
                  'human_recordings_used': False}
        (base/'pose-replay-verification.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
