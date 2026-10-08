"""Independent up/down modules: heights, rotation, contact crossing, validation."""
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
from app import Simulation, ROOT
from layout import validate_layout, preview_xml
from navigation import Navigator


def example():
    return {'schema': 1, 'blocks': [{'id': 'block_1', 'x': 4, 'y': 3},
            {'id': 'block_2', 'x': 4, 'y': -3}], 'slopes': [
            {'id': 'up_1', 'kind': 'up', 'x': 1.5, 'y': -1, 'yaw': 0},
            {'id': 'down_1', 'kind': 'down', 'x': 1.5, 'y': 1, 'yaw': 0}]}


def main():
    data = validate_layout(example())
    assert validate_layout(json.loads(json.dumps(data))) == data
    root = ET.fromstring(preview_xml(data, ROOT/'scene_fork_tracks.xml'))
    base = root.find("worldbody/body[@name='dozer']")
    base.set('pos', '1.5 -3.2 0.30')
    base.set('euler', '0 0 90')
    sim = Simulation(scene_xml=ET.tostring(root, encoding='unicode'))
    origin = {n: sim.data.body(n).xpos.copy() for n in ('up_1', 'down_1')}
    for name in origin:
        g = sim.model.geom(name+'_landing').id
        assert abs(sim.data.geom_xpos[g, 2]+sim.model.geom_size[g, 2] - 0.44) < 0.001
        assert sim.model.body(name).jntnum[0] == 1
    assert sim.model.neq == 0
    for _ in range(100):
        sim.step(0, 0, 1)
    pilot = Navigator(sim, [[1.5, 3.2]])
    surfaces = {sim.model.geom(n+s).id for n in origin for s in ('_incline','_landing')}
    wheel_bodies = {sim.model.body(n).id for n in ('left_front','left_rear','right_front','right_rear')}
    touched = set()
    max_z = 0
    max_movement = 0
    while not pilot.done:
        before = sim.data.qpos.copy()
        action = pilot.action(sim)
        assert np.array_equal(before, sim.data.qpos)
        sim.step(*action)
        sim.observe()
        max_z = max(max_z, sim.data.body('dozer').xpos[2])
        max_movement = max(max_movement, *(np.linalg.norm(sim.data.body(n).xpos-p) for n,p in origin.items()))
        for c in sim.data.contact:
            a,b = map(int,c.geom)
            for surface,wheel in ((a,b),(b,a)):
                if surface in surfaces and int(sim.model.geom_bodyid[wheel]) in wheel_bodies:
                    touched.add(surface)
    assert pilot.success and max_z > .72 and touched == surfaces, (pilot.reason,max_z,touched)
    assert max_movement < .04, max_movement
    rotated = example()
    rotated['slopes'][0].update(x=0,y=0,yaw=90)
    rotated['slopes'][1].update(x=2,y=0,yaw=90)
    rotated_sim = Simulation(layout=rotated)
    for name in origin:
        g=rotated_sim.model.geom(name+'_landing').id
        assert abs(rotated_sim.data.geom_xpos[g,2]+rotated_sim.model.geom_size[g,2]-.44) < .001
    for change in ({'yaw':45},{'height':.3},{'x':float('nan')},{'y':1}):
        bad=example();bad['slopes'][0].update(change)
        try:validate_layout(bad)
        except ValueError:continue
        raise AssertionError('Invalid part accepted')
    folder=ROOT/'.test-results';folder.mkdir(exist_ok=True)
    (folder/'split-slopes.json').write_text(json.dumps(data),encoding='utf-8')
    print(json.dumps({'success':True,'independent_bodies':2,'contact_surfaces':len(touched),
                      'vehicle_rise_m':max_z-.3,'max_part_movement_m':max_movement,
                      'seconds':round(sim.data.time,2)},indent=2))


if __name__ == '__main__':
    main()
