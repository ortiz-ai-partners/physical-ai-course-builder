"""Continuous two-part construction, crossing, and disturbance rejection."""
import json
import numpy as np
from app import Simulation, ROOT
from slope_assembly import assembly_xml, SlopeAssembly


def new_trial():
    sim=Simulation(scene_xml=assembly_xml(ROOT/'scene_fork_tracks.xml'))
    return sim,SlopeAssembly(sim)


def main():
    sim,pilot=new_trial()
    assert sim.model.neq==0
    def forbidden():raise AssertionError('Reset during construction')
    sim.reset=forbidden
    while not pilot.done:
        before=sim.data.qpos.copy()
        action=pilot.action(sim)
        assert np.array_equal(before,sim.data.qpos)
        time=sim.data.time
        sim.step(*action)
        assert sim.data.time>time
    sim.observe()
    assert pilot.success and len(pilot.results)==2 and len(pilot.touched)==4,pilot.reason
    errors={}
    for name,y in [('up_1',-1),('down_1',1.05)]:
        body=sim.model.body(name)
        position=sim.data.body(name).xpos
        errors[name]=float(np.linalg.norm(position[:2]-[1.5,y]))
        assert errors[name]<.06 and abs(position[2]-.08)<.002
        dof=sim.model.jnt_dofadr[body.jntadr[0]]
        assert np.linalg.norm(sim.data.qvel[dof:dof+6])<.02
        g=sim.model.geom(name+'_landing').id
        assert abs(sim.data.geom_xpos[g,2]+sim.model.geom_size[g,2]-.44)<.002
    assert all(p['lift_m']>.15 and p['carry_m']>2.9 for p in pilot.results)
    report={'success':True,'seconds':round(sim.data.time,2),'placement_errors_m':errors,
            'parts':pilot.results,'vehicle_rise_m':pilot.max_rise,'nominal_gap_m':.05,
            'fresh_ai_inference':False,'controller':'feedback_rules'}
    disturbed,guard=new_trial()
    while not guard.placed and not guard.done:
        disturbed.step(*guard.action(disturbed))
    assert guard.placed
    disturbed.data.xfrc_applied[disturbed.model.body('up_1').id,0]=2000
    for _ in range(20):
        disturbed.step(*guard.action(disturbed))
        if guard.done:break
    assert guard.done and not guard.success and 'disturbed' in guard.reason
    report['placed_part_disturbance_rejected']=True
    folder=ROOT/'.test-results';folder.mkdir(exist_ok=True)
    (folder/'slope-assembly-report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
