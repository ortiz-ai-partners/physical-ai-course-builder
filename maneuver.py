"""Empty-vehicle baseline for future demonstrations and learned policies.

Position targets only; no prescribed final heading or obstacle-avoidance planner.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np


def maneuver_xml(scene_path, target=(-1, 0), yaw=0, blocked=False):
    root = ET.parse(scene_path).getroot()
    world = root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') in ('block_3', 'block_4', 'ball'):
            world.remove(body)
    for name, y in [('block_1', 3), ('block_2', -3)]:
        world.find(f"body[@name='{name}']").set('pos', f'4 {y} 0.26')
    if blocked:
        world.find("body[@name='block_1']").set('pos', '-1.2 0 0.26')
    vehicle = world.find("body[@name='dozer']")
    vehicle.set('pos', '0 0 0.3')
    vehicle.set('quat', f'{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}')
    ET.SubElement(world, 'geom', name='maneuver_target', type='cylinder',
                  pos=f'{target[0]} {target[1]} 0.008', size='.18 .006',
                  rgba='0.2 0.85 0.5 0.6', contype='0', conaffinity='0')
    return ET.tostring(root, encoding='unicode')


def evaluate(target, allow_reverse, yaw=0, blocked=False):
    from app import ROOT, Simulation
    from navigation import Navigator
    sim = Simulation(scene_xml=maneuver_xml(ROOT/'scene_fork_tracks.xml', target, yaw, blocked))
    pilot = Navigator(sim, [target], allow_reverse=allow_reverse)
    length = rotation = 0.0
    reverse_steps = 0
    previous = sim.data.body('dozer').xpos[:2].copy()
    previous_yaw = yaw
    while not pilot.done:
        before = sim.data.qpos.copy()
        action = pilot.action(sim)
        assert np.array_equal(before, sim.data.qpos), 'Controller teleported vehicle'
        reverse_steps += int(action[0] < -.01)
        sim.step(*action)
        sim.observe()
        position = sim.data.body('dozer').xpos[:2].copy()
        length += float(np.linalg.norm(position-previous))
        previous = position
        w, x, y, z = sim.data.body('dozer').xquat
        angle = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        rotation += abs((angle-previous_yaw+math.pi) % (2*math.pi)-math.pi)
        previous_yaw = angle
    return {'target': list(target), 'initial_yaw_rad': yaw,
            'allow_reverse': allow_reverse, 'success': pilot.success,
            'reason': pilot.reason, 'seconds': round(sim.data.time, 2),
            'distance_m': length, 'turn_degrees': math.degrees(rotation),
            'reverse_steps': reverse_steps,
            'position_error_m': float(np.linalg.norm(previous-target)),
            'controller': 'feedback_rules', 'learning_performed': False}
