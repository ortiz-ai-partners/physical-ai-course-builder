"""A free-body lightweight bridge ramp, lifted from the side under its deck.

Provisional test object, not a measured real-world product or load rating.
"""
import xml.etree.ElementTree as ET
import numpy as np
from transport import Transport
from navigation import Navigator
from parts import STANDARD_TOP_HEIGHT


def add_portable_ramp(root, x=-1.5, y=0.0, height=STANDARD_TOP_HEIGHT):
    if height not in (0.2, 0.3, 0.4, STANDARD_TOP_HEIGHT):
        raise ValueError('Portable ramp height must be 0.2, 0.3, 0.4 or the standard 0.44 m.')
    world, asset = root.find('worldbody'), root.find('asset')
    body = ET.SubElement(world, 'body', name='portable_ramp', pos=f'{x} {y} 0.08')
    ET.SubElement(body, 'freejoint', name='ramp_free')
    # 3.6 m along Y, 1 m across X. Central deck has 8 cm fork clearance.
    for name, ends in [('portable_up', [(-1.8, -0.08), (-0.3, -0.08), (-0.3, height-0.08)]),
                       ('portable_down', [(0.3, -0.08), (1.8, -0.08), (0.3, height-0.08)])]:
        vertices = ' '.join(f'{vx} {vy} {z}' for vx in (-0.5, 0.5) for vy, z in ends)
        ET.SubElement(asset, 'mesh', name=name, vertex=vertices)
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, mass='0.9',
                      rgba='0.23 0.65 0.74 1', friction='1.2 0.005 0.0001')
    ET.SubElement(body, 'geom', name='portable_deck', type='box', pos=f'0 0 {(height-0.08)/2}',
                  size=f'0.5 0.3 {(height-0.08)/2}', mass='1.2', rgba='0.97 0.73 0.24 1',
                  friction='1.2 0.005 0.0001')


def portable_xml(scene_path, height=STANDARD_TOP_HEIGHT):
    root = ET.parse(scene_path).getroot()
    world = root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') in ('block_3', 'block_4', 'ball'):
            world.remove(body)
    for name, y in [('block_1', 3.5), ('block_2', -3.5)]:
        world.find(f"body[@name='{name}']").set('pos', f'4 {y} 0.26')
    add_portable_ramp(root, height=height)
    return ET.tostring(root, encoding='unicode')


class PortablePilot:
    def __init__(self, sim):
        self.pilot = Transport(sim, {'x': 1.5, 'y': 0}, 'portable_ramp', 1.28, 2.05)
        self.phase = 'TRANSPORT'
        self.stage = self.phase
        self.reason = ''
        self.done = self.success = False
        self.elapsed = 0
        self.placed = None
        self.max_rise = 0.0
        self.lift_m = self.carry_m = 0.0
        self.contacts = set()
        self.ramp_ids = {sim.model.geom(n).id for n in ('portable_up', 'portable_deck', 'portable_down')}
        self.wheel_bodies = {sim.model.body(n).id for n in
                             ('left_front', 'left_rear', 'right_front', 'right_rear')}
        self.base_z = float(sim.data.body('dozer').xpos[2])
        self.height = float(sim.model.geom_size[sim.model.geom('portable_deck').id, 2]*2+0.08)

    def stop(self, reason):
        self.done = True
        self.stage = 'COMPLETE' if self.success else 'STOPPED'
        self.reason = reason
        return (0, 0, 0)

    def action(self, sim):
        if self.done:
            return (0, 0, 0)
        sim.observe()
        self.elapsed += 1
        if self.elapsed > 20000 or not np.isfinite(sim.data.qpos).all():
            return self.stop('Time limit or invalid state')
        ramp = sim.data.body('portable_ramp')
        if self.placed is not None and np.linalg.norm(ramp.xpos - self.placed) > 0.04:
            return self.stop('Placed ramp moved too far')
        if self.phase == 'TRANSPORT':
            action = self.pilot.action(sim)
            self.stage = 'TRANSPORT: ' + self.pilot.stage
            if self.pilot.done:
                if not self.pilot.success:
                    return self.stop(self.pilot.reason)
                self.lift_m, self.carry_m = self.pilot.max_rise, self.pilot.raised_travel
                self.placed = ramp.xpos.copy()
                self.phase = 'RAISE'
            return action
        if self.phase == 'RAISE':
            self.stage = 'RAISE FORKS'
            if sim.lift_target < 0.49:
                return (0, 0, 1)
            self.phase = 'APPROACH'
            self.pilot = Navigator(sim, [[-1, 0], [-1, -3], [1.5, -3]])
        if self.phase in ('APPROACH', 'CROSS'):
            action = self.pilot.action(sim)
            self.stage = self.phase + ': ' + self.pilot.stage
            if self.phase == 'CROSS':
                self.max_rise = max(self.max_rise, float(sim.data.body('dozer').xpos[2]) - self.base_z)
                for c in sim.data.contact:
                    a, b = map(int, c.geom)
                    for ground, wheel in ((a, b), (b, a)):
                        if ground in self.ramp_ids and int(sim.model.geom_bodyid[wheel]) in self.wheel_bodies:
                            self.contacts.add(ground)
            if self.pilot.done:
                if not self.pilot.success:
                    return self.stop(self.pilot.reason)
                if self.phase == 'APPROACH':
                    self.phase = 'CROSS'
                    self.pilot = Navigator(sim, [[1.5, 3]])
                else:
                    self.success = self.max_rise > self.height*0.8 and self.contacts == self.ramp_ids
                    return self.stop(f'Lift {self.lift_m:.3f} m, carry {self.carry_m:.3f} m, climb {self.max_rise:.3f} m')
            return action
        raise RuntimeError(self.phase)
