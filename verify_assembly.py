"""No-reset, no-coordinate-write integration checks for restricted gate assembly."""
import copy
import json
import numpy as np
from app import Simulation
from ai_plan import load_plan
from assembly import Assembly, validate_gate
from layout import ROOT


def main():
    reports=[]
    for x, y in [(1.5,1.0),(2.75,1.25)]:
        plan=load_plan(ROOT/'examples'/'ortiz-gate-plan.json')
        plan['layout']['blocks'][0].update(x=x,y=y)
        plan['layout']['blocks'][1].update(x=x,y=-y)
        plan['waypoints']=[[-1.5,0],[0,0],[x+1.0,0]]
        sim=Simulation(layout=plan['layout'],construction='gate')
        pilot=Assembly(sim,plan)
        for name in ('block_1','block_2'):
            assert abs(sim.data.body(name).xpos[0]+1.5)<1e-6
        def forbidden_reset():
            raise AssertionError('Scene reset during construction')
        sim.reset=forbidden_reset
        last_time=sim.data.time
        while not pilot.done:
            qpos=sim.data.qpos.copy()
            action=pilot.action(sim)
            np.testing.assert_array_equal(sim.data.qpos,qpos)
            sim.step(*action)
            assert sim.data.time>last_time
            last_time=sim.data.time
        assert pilot.success,(x,y,pilot.reason,pilot.phase)
        assert len(pilot.results)==2 and all(r['lift_m']>.15 for r in pilot.results)
        assert sim.model.neq==0
        errors=[]
        for target in plan['layout']['blocks']:
            error=float(np.linalg.norm(sim.data.body(target['id']).xpos[:2]-[target['x'],target['y']]))
            assert error<.06
            errors.append(error)
        reports.append({'gate':[x,y],'placement_errors_m':errors,'seconds':pilot.elapsed*.02,'success':True})
    invalid=copy.deepcopy(plan)
    invalid['layout']['blocks'][1]['x']-=.25
    try:
        validate_gate(invalid)
        raise AssertionError('Unsupported gate accepted')
    except ValueError:
        pass
    print(json.dumps(reports,indent=2))


if __name__=='__main__':
    main()
