"""Independent slope pickup, carry, release, and missed-pickup failure."""
import json
import numpy as np
from app import Simulation, ROOT
from slope_transport import slope_transport_xml, slope_pilot


def check(kind, target_x):
    sim = Simulation(scene_xml=slope_transport_xml(ROOT/'scene_fork_tracks.xml', kind))
    pilot = slope_pilot(sim, kind, target_x)
    name = kind+'_1'
    rail = sim.model.geom(name+'_pickup').id
    forks = {sim.model.geom(n).id for n in ('fork_left','fork_right')}
    contact_seen = False
    assert sim.model.neq == 0
    def forbidden():
        raise AssertionError('Unexpected reset')
    sim.reset = forbidden
    while not pilot.done:
        before = sim.data.qpos.copy()
        action = pilot.action(sim)
        assert np.array_equal(before, sim.data.qpos)
        previous = sim.data.time
        sim.step(*action)
        assert sim.data.time > previous
        for c in sim.data.contact:
            a,b=map(int,c.geom)
            contact_seen |= (a==rail and b in forks) or (b==rail and a in forks)
    sim.observe()
    position = sim.data.body(name).xpos
    error = float(np.linalg.norm(position[:2]-[target_x,0]))
    assert pilot.success and contact_seen and pilot.max_rise > .15 and error < .06, pilot.reason
    assert abs(position[2]-.08) < .002
    body = sim.model.body(name)
    dof = sim.model.jnt_dofadr[body.jntadr[0]]
    assert np.linalg.norm(sim.data.qvel[dof:dof+6]) < .02
    top = sim.model.geom(name+'_landing').id
    assert abs(sim.data.geom_xpos[top,2]+sim.model.geom_size[top,2]-.44) < .002
    return {'kind':kind,'target_x':target_x,'success':True,'lift_m':pilot.max_rise,
            'carry_m':pilot.raised_travel,'error_m':error,'seconds':round(sim.data.time,2)}


def main():
    results = [check(kind,x) for kind in ('up','down') for x in (.5,1.5,2.5)]
    sim = Simulation(scene_xml=slope_transport_xml(ROOT/'scene_fork_tracks.xml','up'))
    for name in ('fork_left','fork_right'):
        g=sim.model.geom(name).id
        sim.model.geom_contype[g]=sim.model.geom_conaffinity[g]=0
    pilot=slope_pilot(sim,'up')
    while not pilot.done:
        sim.step(*pilot.action(sim))
    assert not pilot.success and 'did not lift' in pilot.reason, pilot.reason
    report={'cases':results,'missed_pickup_rejected':True}
    folder=ROOT/'.test-results';folder.mkdir(exist_ok=True)
    (folder/'slope-transport-report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
