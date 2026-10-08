"""Carry a free-body ramp and cross the placed object in one simulation."""
import json
import numpy as np
from app import Simulation, ROOT
from portable_ramp import PortablePilot


def main():
    sim = Simulation(portable_ramp=True)
    pilot = PortablePilot(sim)
    assert sim.model.neq == 0, 'Ramp must not be welded to floor or forks'
    assert abs(sim.model.body('portable_ramp').mass[0] - 3) < 1e-8
    def forbidden():
        raise AssertionError('Reset during construction and crossing')
    sim.reset = forbidden
    while not pilot.done:
        before = sim.data.qpos.copy()
        action = pilot.action(sim)
        assert np.array_equal(before, sim.data.qpos), 'Direct pose assignment'
        previous_time = sim.data.time
        sim.step(*action)
        assert sim.data.time > previous_time
    sim.observe()
    ramp = sim.data.body('portable_ramp')
    error = float(np.linalg.norm(ramp.xpos[:2] - [1.5, 0]))
    assert pilot.success and pilot.lift_m > 0.15 and pilot.carry_m > 2.9, pilot.reason
    assert pilot.max_rise > 0.24 and len(pilot.contacts) == 3
    assert error < 0.06
    assert abs(sim.data.body('dozer').xpos[2] - pilot.base_z) < 0.02
    j = sim.model.body('portable_ramp').jntadr[0]
    dof = sim.model.jnt_dofadr[j]
    assert np.linalg.norm(sim.data.qvel[dof:dof+6]) < 0.02
    report = {'success': pilot.success, 'controller': 'feedback_rules', 'fresh_ai_inference': False,
              'simulation_seconds': round(sim.data.time, 2), 'lift_m': pilot.lift_m,
              'carry_m': pilot.carry_m, 'vehicle_climb_m': pilot.max_rise,
              'ramp_placement_error_m': error, 'final_state': sim.observe()}
    folder = ROOT / '.test-results'
    folder.mkdir(exist_ok=True)
    (folder / 'portable-ramp-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k != 'final_state'}, indent=2))


if __name__ == '__main__':
    main()
