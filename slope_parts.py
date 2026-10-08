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
    # Thin rigid deck with an open underside and a pickup rail near the COM.
    # Top surface remains the same 1.5 m incline + 0.5 m landing.
    ends = [(-1, -0.08), (-0.86, -0.08),
            (0.5, STANDARD_TOP_HEIGHT-0.12), (0.5, STANDARD_TOP_HEIGHT-0.08)]
    vertices = ' '.join(f'{x} {sign*y} {z}' for x in (-0.5, 0.5) for y, z in ends)
    ET.SubElement(asset, 'mesh', name=name+'_mesh', vertex=vertices)
    ET.SubElement(body, 'geom', name=name+'_incline', type='mesh', mesh=name+'_mesh',
                  mass='0.7', rgba='0.23 0.65 0.74 1', friction='1.2 0.005 0.0001')
    ET.SubElement(body, 'geom', name=name+'_landing', type='box',
                  pos=f'0 {sign*0.75} {STANDARD_TOP_HEIGHT-0.10}', size='0.5 0.25 0.02',
                  mass='0.3', rgba='0.97 0.73 0.24 1', friction='1.2 0.005 0.0001')
    for x in (-0.43, 0.43):
        ET.SubElement(body, 'geom', type='box', pos=f'{x} {sign*0.85} 0.12',
                      size='0.045 0.08 0.20', mass='0.15', rgba='0.18 0.3 0.32 1')
    ET.SubElement(body, 'geom', name=name+'_pickup', type='box',
                  pos=f'0 {sign*0.25} 0.025', size='0.45 0.20 0.025',
                  mass='0.2', rgba='0.97 0.73 0.24 1', friction='1.2 0.005 0.0001')
