"""Validated target layouts. Targets are separate from construction actions."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GRID = 0.25


def validate_layout(value):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('配置データの形式が違います。')
    blocks = value.get('blocks')
    if not isinstance(blocks, list) or len(blocks) != 2:
        raise ValueError('この版では箱を2個配置してください。')
    result = []
    for i, block in enumerate(blocks, 1):
        if not isinstance(block, dict) or block.get('id') != f'block_{i}':
            raise ValueError('箱の番号が違います。')
        x, y = block.get('x'), block.get('y')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            raise ValueError('位置は有限の数値にしてください。')
        if not (-1 <= x <= 4 and -3 <= y <= 3):
            raise ValueError('箱は設計エリア内に置いてください。')
        if any(abs(v / GRID - round(v / GRID)) > 1e-7 for v in (x, y)):
            raise ValueError('箱は25cmのマス目に合わせてください。')
        if block.get('yaw', 0) != 0:
            raise ValueError('この版では箱の向きは固定です。')
        result.append({'id': f'block_{i}', 'x': float(x), 'y': float(y), 'yaw': 0})
    a, b = result
    if abs(a['x'] - b['x']) < 0.44 and abs(a['y'] - b['y']) < 0.48:
        raise ValueError('箱が重なっています。離して置いてください。')
    ramps = value.get('ramps', [])
    if not isinstance(ramps, list) or len(ramps) > 1:
        raise ValueError('この版で配置できる坂道は1個です。')
    checked = []
    for ramp in ramps:
        if not isinstance(ramp, dict) or ramp.get('id') != 'ramp_1':
            raise ValueError('坂道の番号が違います。')
        x, y, h = ramp.get('x'), ramp.get('y'), ramp.get('height')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y, h)):
            raise ValueError('坂道の位置と高さは有限の数値にしてください。')
        if not (-1 <= x <= 4 and -2 <= y <= 2):
            raise ValueError('坂道が設計エリアからはみ出しています。')
        if any(abs(v / GRID - round(v / GRID)) > 1e-7 for v in (x, y)):
            raise ValueError('坂道は25cmのマス目に合わせてください。')
        if h not in (0.2, 0.3, 0.4) or ramp.get('yaw', 0) != 0:
            raise ValueError('運搬用の坂道は高さ20・30・40cm、向き固定です。')
        for block in result:
            if abs(x - block['x']) < 0.72 and abs(y - block['y']) < 2.04:
                raise ValueError('坂道と箱が重なっています。離して置いてください。')
        checked.append({'id': 'ramp_1', 'x': float(x), 'y': float(y), 'height': float(h), 'yaw': 0})
    output = {'schema': 1, 'kind': 'target_layout', 'units': 'm',
              'grid_m': GRID, 'blocks': result}
    if checked:
        output['ramps'] = checked
    return output


def load_layout(path):
    return validate_layout(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def preview_xml(layout, scene_path, construction=False):
    """Render the desired END state, not an executed construction result."""
    layout = validate_layout(layout)
    if construction and layout.get('ramps'):
        raise ValueError('坂道付き配置の施工はまだ未対応です。3Dプレビューで確認してください。')
    root = ET.parse(scene_path).getroot()
    world = root.find('worldbody')
    if layout.get('ramps'):
        from portable_ramp import add_portable_ramp
        ramp = layout['ramps'][0]
        add_portable_ramp(root, x=ramp['x'], y=ramp['y'], height=ramp['height'])
        # Preview has no driving. Park the vehicle outside every allowed ramp.
        world.find("body[@name='dozer']").set('pos', '-4.5 3.9 0.30')
    for body in list(world.findall('body')):
        if body.get('name') in (('block_2', 'block_3', 'block_4', 'ball') if construction is True else ('block_3', 'block_4', 'ball')):
            world.remove(body)
    for block in layout['blocks']:
        body = world.find(f"body[@name='{block['id']}']")
        if body is not None:
            x = -1.5 if construction else block['x']
            body.set('pos', f"{x} {block['y']} 0.26")
    if construction:
        target = layout['blocks'][0]
        world.find("body[@name='dozer']").set('pos', f"-3 {target['y']} 0.30")
        ET.SubElement(world, 'geom', {'name': 'build_target', 'type': 'box',
            'pos': f"{target['x']} {target['y']} 0.006", 'size': '0.27 0.29 0.005',
            'contype': '0', 'conaffinity': '0', 'rgba': '0.95 0.35 0.3 0.35'})
    return ET.tostring(root, encoding='unicode')
