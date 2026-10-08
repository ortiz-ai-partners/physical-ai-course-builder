"""Independent movable up/down slope modules; no floor welds."""
import xml.etree.ElementTree as ET
from parts import STANDARD_TOP_HEIGHT


def half_extents(part):
    return (0.5, 1.0) if part.get('yaw', 0) in (0, 180) else (1.0, 0.5)


def add_slope(root, part):
    name = part['id']
    sign = 1 if part['kind'] == 'up' else -1
    world, asset = root.find('worldbody'), root.find('asset')
    body = ET.SubElement(world, 'body', name=name, pos=f"{part['x']} {part['y']} 0.08",
                         euler=f"0 0 {part.get('yaw', 0)}")
    ET.SubElement(body, 'freejoint', name=name+'_free')
    # 1.5 m incline + 0.5 m landing; fork clearance below the landing.
    ends = [(-1, -0.08), (0.5, -0.08), (0.5, STANDARD_TOP_HEIGHT-0.08)]
    vertices = ' '.join(f'{x} {sign*y} {z}' for x in (-0.5, 0.5) for y, z in ends)
    ET.SubElement(asset, 'mesh', name=name+'_mesh', vertex=vertices)
    ET.SubElement(body, 'geom', name=name+'_incline', type='mesh', mesh=name+'_mesh',
                  mass='0.9', rgba='0.23 0.65 0.74 1', friction='1.2 0.005 0.0001')
    half = (STANDARD_TOP_HEIGHT-0.08)/2
    ET.SubElement(body, 'geom', name=name+'_landing', type='box',
                  pos=f'0 {sign*0.75} {half}', size=f'0.5 0.25 {half}',
                  mass='0.6', rgba='0.97 0.73 0.24 1', friction='1.2 0.005 0.0001')
