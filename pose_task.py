"""Pose-goal teleoperation task and separate, goal-aware demonstration records."""
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from maneuver import maneuver_xml

CASES = {
    'back': {'target': [-1, 0, 0], 'initial_yaw': 0, 'split': 'train'},
    'front': {'target': [1, 0, 0], 'initial_yaw': 0, 'split': 'train'},
    'rear-left': {'target': [-1, .5, 0], 'initial_yaw': 0, 'split': 'train'},
    'rear-right-eval': {'target': [-1, -.5, 0], 'initial_yaw': 0, 'split': 'evaluation'},
    'rotated-eval': {'target': [0, -1, math.pi/2], 'initial_yaw': math.pi/2, 'split': 'evaluation'},
}
FEATURE_NAMES = ['goal_forward_m', 'goal_left_m', 'sin_goal_yaw_error',
                 'cos_goal_yaw_error', 'forward_speed_m_s', 'left_speed_m_s', 'yaw_speed_rad_s']


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def features(state, target):
    pose = state['objects']['dozer']
    w, x, y, z = pose['quaternion_wxyz']
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    dx, dy = np.asarray(target[:2])-pose['position'][:2]
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = state['qvel'][:2]
    error = wrap(target[2]-yaw)
    return [float(c*dx+s*dy), float(-s*dx+c*dy), math.sin(error), math.cos(error),
            float(c*vx+s*vy), float(-s*vx+c*vy), float(state['qvel'][5])]


def pose_xml(scene_path, case):
    config = CASES[case]
    target = config['target']
    root = ET.fromstring(maneuver_xml(scene_path, target[:2], config['initial_yaw']))
    world = root.find('worldbody')
    # Flat direction arrow; no contact force or hidden constraint.
    c, s = math.cos(target[2]), math.sin(target[2])
    def segment(name, a, b):
        ET.SubElement(world, 'geom', name=name, type='capsule', size='.025',
                      fromto=f'{a[0]} {a[1]} .025 {b[0]} {b[1]} .025',
                      rgba='.15 .85 .55 1', contype='0', conaffinity='0')
    start = np.array(target[:2]); end = start + np.array([c, s])*.95
    segment('goal_heading', start, end)
    for sign in (-1, 1):
        tail = end - .2*np.array([c, s]) + sign*.13*np.array([-s, c])
        segment('goal_arrow_'+str(sign), tail, end)
    return ET.tostring(root, encoding='unicode')


class PoseTask:
    def __init__(self, sim, case):
        self.case = case
        self.config = CASES[case]
        self.target = list(self.config['target'])
        self.position_tolerance = .08
        self.angle_tolerance = math.radians(8)
        self.reset(sim)

    def reset(self, sim):
        self.done = self.success = False
        self.reason = 'Match position and heading; stop for one second.'
        self.hold_seconds = 0.0
        self.started = self.last_time = float(sim.data.time)
        self.metrics = self.measure(sim.observe())

    def measure(self, state):
        f = features(state, self.target)
        return {'position_error_m': math.hypot(f[0], f[1]),
                'heading_error_deg': abs(math.degrees(math.atan2(f[2], f[3]))),
                'speed_m_s': math.hypot(f[4], f[5]), 'angular_speed_rad_s': abs(f[6])}

    def update(self, sim):
        if self.done: return
        state = sim.observe()
        self.metrics = self.measure(state)
        dt = max(0, state['time']-self.last_time)
        self.last_time = state['time']
        # Vehicle root includes its wheels and forks. Ignore floor contact only.
        vehicle = sim.model.body('dozer').id
        floor = sim.model.geom('floor').id
        for contact in sim.data.contact:
            ga, gb = map(int, contact.geom)
            if floor in (ga, gb): continue
            roots = [int(sim.model.body_rootid[sim.model.geom_bodyid[g]]) for g in (ga, gb)]
            if (roots[0] == vehicle) != (roots[1] == vehicle):
                self.done = True; self.reason = 'Contact with an obstacle'; return
        w, x, y, z = state['objects']['dozer']['quaternion_wxyz']
        if not np.isfinite(sim.data.qpos).all() or 1-2*(x*x+y*y) < .7:
            self.done = True; self.reason = 'Invalid state or excessive tilt'; return
        m = self.metrics
        matched = (m['position_error_m'] <= self.position_tolerance and
                   m['heading_error_deg'] <= math.degrees(self.angle_tolerance) and
                   m['speed_m_s'] < .03 and m['angular_speed_rad_s'] < .04)
        self.hold_seconds = self.hold_seconds+dt if matched else 0.0
        if self.hold_seconds >= 1.0-1e-8:
            self.done = self.success = True
            self.reason = 'Position + heading matched; stopped for one second'
        elif state['time']-self.started >= 120:
            self.done = True; self.reason = 'Time limit (120 seconds)'


class PoseRecorder:
    def __init__(self, task, scene_xml, folder, source='keyboard_teleoperation'):
        self.task, self.scene_xml, self.folder = task, scene_xml, Path(folder)
        self.source = source
        self.file = self.path = None
        self.frames = 0
        self.saved_success = False
        from pose_dataset import collection_summary
        self.collection_counts = collection_summary(self.folder)['unique_successes']

    def write(self, row):
        if row.get('type') == 'transition':
            row['observation'] = features(row['state'], self.task.target)
            row['next_observation'] = features(row['next_state'], self.task.target)
            row['goal'] = self.task.target
        self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        self.file.flush()

    def start(self, sim):
        if self.file or self.task.done: return
        # A success label must be supported by a full second inside this file.
        self.task.hold_seconds = 0.0
        import mujoco
        self.folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path = self.folder/f'pose_{self.task.case}_{stamp}.jsonl'
        self.file = self.path.open('x', encoding='utf-8')
        self.frames = 0
        self.saved_success = False
        state_type = mujoco.mjtState.mjSTATE_INTEGRATION
        integration = np.empty(mujoco.mj_stateSize(sim.model, state_type))
        mujoco.mj_getState(sim.model, sim.data, integration, state_type)
        self.write({'type': 'header', 'schema': 'pose-demo-v1', 'case': self.task.case,
                    'split': self.task.config['split'], 'source': self.source,
                    'engine_version': mujoco.__version__, 'control_dt': .02,
                    'goal': self.task.target, 'feature_names': FEATURE_NAMES,
                    'actions': ['forward', 'turn_left', 'lift_up'],
                    'scene_sha256': hashlib.sha256(self.scene_xml.encode()).hexdigest(),
                    'scene_xml': self.scene_xml, 'initial_state': sim.observe(),
                    'integration_state': integration.tolist(),
                    'success_criteria': {'position_m': .08, 'heading_deg': 8,
                                         'speed_m_s': .03, 'yaw_speed_rad_s': .04, 'hold_seconds': 1}})

    def stop(self, reason='user'):
        if not self.file: return
        self.saved_success = self.task.success
        self.write({'type': 'end', 'frames': self.frames, 'reason': reason,
                    'success': self.task.success, 'task_reason': self.task.reason,
                    'metrics': self.task.metrics})
        self.file.close(); self.file = None
        if self.saved_success and self.source == 'keyboard_teleoperation' and self.task.config['split'] == 'train':
            self.collection_counts[self.task.case] += 1
