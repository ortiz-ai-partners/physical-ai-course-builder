"""Animated track shoes; ground contact remains the four-wheel proxy.

The links are visual-only: no articulated chain or continuous belt dynamics.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

RADIUS = 0.168
HALF_LENGTH = 0.29
LENGTH = 4 * HALF_LENGTH + 2 * math.pi * RADIUS
COUNT = 38


def shoe_pose(distance, side):
    s = distance % LENGTH
    straight = 2 * HALF_LENGTH
    arc = math.pi * RADIUS
    if s < straight:
        x, z, angle = -HALF_LENGTH + s, RADIUS, 0
    elif s < straight + arc:
        theta = math.pi / 2 - (s - straight) / RADIUS
        x, z = HALF_LENGTH + RADIUS * math.cos(theta), RADIUS * math.sin(theta)
        angle = math.pi / 2 - theta
    elif s < 2 * straight + arc:
        x, z, angle = HALF_LENGTH - (s - straight - arc), -RADIUS, math.pi
    else:
        theta = -math.pi / 2 - (s - 2 * straight - arc) / RADIUS
        x, z = -HALF_LENGTH + RADIUS * math.cos(theta), RADIUS * math.sin(theta)
        angle = math.pi / 2 - theta
    return [x, side * 0.34, z - 0.12], [math.cos(angle / 2), 0, math.sin(angle / 2), 0]


def build_scenes():
    root = Path(__file__).resolve().parent
    for source, target in [('scene.xml', 'scene_tracks.xml'), ('scene_fork.xml', 'scene_fork_tracks.xml')]:
        tree = ET.parse(root / source)
        body = tree.getroot().find(".//body[@name='dozer']")
        for side, label in [(1, 'left'), (-1, 'right')]:
            ET.SubElement(body, 'geom', {'class': 'decor', 'type': 'box',
                          'pos': f'0 {side * .34} -.12', 'size': '.29 .064 .125',
                          'rgba': '.065 .078 .09 1'})
            for x in [-.13, .13]:
                ET.SubElement(body, 'geom', {'class': 'decor', 'type': 'cylinder',
                              'pos': f'{x} {side * .409} -.14', 'size': '.09 .006',
                              'quat': '.70710678 .70710678 0 0', 'material': 'yellow'})
            for i in range(COUNT):
                pos, quat = shoe_pose(i * LENGTH / COUNT, side)
                ET.SubElement(body, 'geom', {'name': f'track_{label}_{i}', 'class': 'decor',
                              'type': 'box', 'pos': ' '.join(map(str, pos)),
                              'quat': ' '.join(map(str, quat)), 'size': '.025 .082 .012',
                              'rgba': '.12 .145 .16 1'})
        ET.indent(tree, space='  ')
        tree.write(root / target, encoding='unicode')


class TrackAnimation:
    def __init__(self, model):
        self.sides = []
        for side, label, front, rear in [(1, 'left', 'lf', 'lr'), (-1, 'right', 'rf', 'rr')]:
            self.sides.append((side, [model.geom(f'track_{label}_{i}').id for i in range(COUNT)],
                               int(model.joint(front).qposadr[0]), int(model.joint(rear).qposadr[0])))

    def update(self, model, data):
        for side, ids, front, rear in self.sides:
            distance = 0.18 * (data.qpos[front] + data.qpos[rear]) / 2
            for i, geom_id in enumerate(ids):
                pos, quat = shoe_pose(i * LENGTH / COUNT + distance, side)
                model.geom_pos[geom_id] = pos
                model.geom_quat[geom_id] = quat


if __name__ == '__main__':
    build_scenes()
