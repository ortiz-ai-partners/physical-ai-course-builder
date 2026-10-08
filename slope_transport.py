"""Straight-lane single slope transport with an offset pickup rail."""
import xml.etree.ElementTree as ET
from slope_parts import add_slope
from transport import Transport


def slope_transport_xml(scene_path, kind='up'):
    if kind not in ('up', 'down'):
        raise ValueError('Slope kind must be up or down')
    root = ET.parse(scene_path).getroot()
    world = root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') in ('block_3', 'block_4', 'ball'):
            world.remove(body)
    for name, y in [('block_1', 3), ('block_2', -3)]:
        world.find(f"body[@name='{name}']").set('pos', f'4 {y} 0.26')
    offset = 0.25 if kind == 'up' else -0.25
    world.find("body[@name='dozer']").set('pos', f'-3 {offset} 0.30')
    add_slope(root, {'id': kind+'_1', 'kind': kind, 'x': -1.5, 'y': 0, 'yaw': 0})
    return ET.tostring(root, encoding='unicode')


def slope_pilot(sim, kind='up', target_x=1.5):
    if kind not in ('up', 'down'):
        raise ValueError('Slope kind must be up or down')
    return Transport(sim, {'x': target_x, 'y': 0}, kind+'_1',
                     pickup_distance=1.28, release_distance=2.05,
                     lane_y=0.25 if kind == 'up' else -0.25)
