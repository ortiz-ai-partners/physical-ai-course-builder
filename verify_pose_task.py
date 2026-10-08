"""Physical goal scoring, held-out labels, and goal-aware record validation."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from app import ROOT, Simulation
from pose_task import PoseTask, PoseRecorder, pose_xml, CASES
from pose_dataset import inspect_episode


def main():
    folder = ROOT/'.test-results/pose'; folder.mkdir(parents=True, exist_ok=True)
    xml = pose_xml(ROOT/'scene_fork_tracks.xml', 'back')
    sim = Simulation(scene_xml=xml); task = PoseTask(sim, 'back')
    recorder = PoseRecorder(task, xml, folder, source='scripted_test')
    recorder.start(sim)
    for _ in range(1200):
        before = sim.observe()
        x = before['objects']['dozer']['position'][0]
        action = [-.2 if x > -.97 else 0, 0, 0]
        controls = sim.step(*action); task.update(sim)
        recorder.write({'type': 'transition', 'step': recorder.frames, 'state': before,
                        'action': action, 'actuator_command': controls.tolist(), 'next_state': sim.observe()})
        recorder.frames += 1
        if task.done: break
    assert task.success, task.metrics
    recorder.stop('task_success')
    result = inspect_episode(recorder.path)
    assert result['success'] and not result['eligible_for_training']
    # Missing trailer and false-success labels must not silently enter training.
    lines = recorder.path.read_text(encoding='utf-8').splitlines()
    bad = folder/'incomplete.jsonl'; bad.write_text('\n'.join(lines[:-1]), encoding='utf-8')
    try: inspect_episode(bad); raise AssertionError('Incomplete accepted')
    except ValueError: pass
    rows = [json.loads(line) for line in lines[:2]]
    rows.append({'type': 'end', 'frames': 1, 'success': True})
    bad.write_text('\n'.join(json.dumps(r) for r in rows), encoding='utf-8')
    try: inspect_episode(bad); raise AssertionError('False success accepted')
    except ValueError: pass
    # Initial placement is a test fixture, not a controller moving the robot.
    root = ET.fromstring(xml)
    car = root.find("worldbody/body[@name='dozer']")
    car.set('pos', '-1 0 .3'); car.set('quat', '.7071067812 0 0 .7071067812')
    wrong = Simulation(scene_xml=ET.tostring(root, encoding='unicode')); check = PoseTask(wrong, 'back')
    for _ in range(100): wrong.step(); check.update(wrong)
    assert check.metrics['position_error_m'] < .08 and check.metrics['heading_error_deg'] > 80
    assert not check.success and check.hold_seconds == 0
    car.set('quat', '1 0 0 0')
    at_goal = Simulation(scene_xml=ET.tostring(root, encoding='unicode')); dwell = PoseTask(at_goal, 'back')
    for _ in range(49): at_goal.step(); dwell.update(at_goal)
    assert not dwell.success
    at_goal.step(); dwell.update(at_goal); assert dwell.success
    for case in CASES:
        trial = Simulation(scene_xml=pose_xml(ROOT/'scene_fork_tracks.xml', case))
        p = PoseTask(trial, case)
        assert not p.done and np.isfinite(list(p.metrics.values())).all()
    report = {'physical_back_trial': result, 'final_errors': task.metrics,
              'wrong_heading_rejected': True, 'one_second_dwell_required': True,
              'incomplete_and_false_labels_rejected': True, 'cases_load': len(CASES),
              'human_demonstrations_collected': 0, 'training_started': False}
    (ROOT/'.test-results/pose-task-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
