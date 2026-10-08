"""Static contact terrain and a feedback-controlled crossing experiment.

Provisional dimensions, not a measured model of the sports club.
"""
import math
import xml.etree.ElementTree as ET
from navigation import Navigator


def add_ramp(root, height=0.35, x=0.5, y=0.0):
    """Append a fixed 5 x 1.8 m ramp, centered at (x, y)."""
    if type(height) not in (int, float) or not math.isfinite(height) or not 0.15 <= height <= 0.5:
        raise ValueError('Ramp height must be between 0.15 and 0.5 m.')
    world, asset = root.find('worldbody'), root.find('asset')
    # Closed convex triangular prisms: the mesh itself is the collision surface.
    for name, ends, color in [
        ('ramp_up', [(-2, 0), (0, 0), (0, height)], '0.23 0.65 0.74 1'),
        ('ramp_down', [(1, 0), (3, 0), (1, height)], '0.23 0.65 0.74 1')]:
        vertices = ' '.join(f'{vx + x - 0.5} {vy + y} {z}' for vy in (-0.9, 0.9) for vx, z in ends)
        ET.SubElement(asset, 'mesh', name=name, vertex=vertices)
        ET.SubElement(world, 'geom', name=name, type='mesh', mesh=name, rgba=color,
                      friction='1.0 0.005 0.0001')
    ET.SubElement(world, 'geom', name='ramp_top', type='box',
                  pos=f'{x} {y} {height / 2}', size=f'0.5 0.9 {height / 2}',
                  rgba='0.97 0.73 0.24 1', friction='1.0 0.005 0.0001')


def ramp_xml(scene_path, height=0.35):
    root = ET.parse(scene_path).getroot()
    world = root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') in ('block_3', 'block_4', 'ball'):
            world.remove(body)
    for name, y in [('block_1', 3), ('block_2', -3)]:
        world.find(f"body[@name='{name}']").set('pos', f'0 {y} 0.26')
    world.find("body[@name='dozer']").set('pos', '-3.8 0 0.30')
    add_ramp(root, height)
    return ET.tostring(root, encoding='unicode')


class RampPilot:
    def __init__(self, sim):
        self.nav = Navigator(sim, [[4.0, 0]])
        self.done = self.success = False
        self.stage, self.reason = 'RAISE FORKS', ''
        self.elapsed = 0
        self.initial_z = float(sim.data.body('dozer').xpos[2])
        self.max_rise = 0.0
        self.wheel_contacts = set()
        self.height = float(sim.model.geom_size[sim.model.geom('ramp_top').id, 2] * 2)
        self.terrain_ids = {sim.model.geom(n).id for n in ('ramp_up', 'ramp_top', 'ramp_down')}
        self.wheel_bodies = {sim.model.body(n).id for n in
                             ('left_front', 'left_rear', 'right_front', 'right_rear')}

    def action(self, sim):
        if self.done:
            return (0, 0, 0)
        self.elapsed += 1
        sim.observe()
        self.max_rise = max(self.max_rise, float(sim.data.body('dozer').xpos[2]) - self.initial_z)
        for contact in sim.data.contact:
            a, b = map(int, contact.geom)
            for terrain, wheel in ((a, b), (b, a)):
                if terrain in self.terrain_ids and int(sim.model.geom_bodyid[wheel]) in self.wheel_bodies:
                    self.wheel_contacts.add(terrain)
        if self.elapsed <= 100:
            return (0, 0, 1)
        action = self.nav.action(sim)
        self.stage = self.nav.stage
        if self.nav.done:
            self.done = True
            self.success = bool(self.nav.success and self.max_rise > self.height * 0.8
                                and self.wheel_contacts == self.terrain_ids)
            self.stage = 'COMPLETE' if self.success else 'STOPPED'
            self.reason = (f'Rise {self.max_rise:.3f} m; wheel contact on {len(self.wheel_contacts)}/3 surfaces. '
                           + self.nav.reason)
        return action
