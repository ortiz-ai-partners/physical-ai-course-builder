"""Carry a free-body ramp and cross the placed object in one simulation."""
import json
import numpy as np
from app import Simulation, ROOT
from portable_ramp import PortablePilot
from parts import STANDARD_TOP_HEIGHT


def main():
    sim = Simulation(portable_ramp=True)
    pilot = PortablePilot(sim)
    def top_of_geom(g):
        return float(sim.data.geom_xpos[g, 2] + sim.model.geom_size[g, 2])
    deck = sim.model.geom('portable_deck').id
    initial_top = top_of_geom(deck)
    for name in ('block_1', 'block_2'):
        g = int(sim.model.body(name).geomadr[0])
        assert abs(top_of_geom(g) - initial_top) < 0.001, 'Part top heights do not match'
    assert abs(initial_top - STANDARD_TOP_HEIGHT) < 0.001
    original = Simulation()
    original.observe()
    for name in ('block_1', 'block_2', 'block_3', 'block_4'):
        g = int(original.model.body(name).geomadr[0])
        top = original.data.geom_xpos[g, 2] + original.model.geom_size[g, 2]
        assert abs(top - STANDARD_TOP_HEIGHT) < 0.001, name
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
    assert pilot.max_rise > STANDARD_TOP_HEIGHT - 0.02 and len(pilot.contacts) == 3
    assert abs(top_of_geom(deck) - STANDARD_TOP_HEIGHT) < 0.002
    assert error < 0.06
    assert abs(sim.data.body('dozer').xpos[2] - pilot.base_z) < 0.02
    j = sim.model.body('portable_ramp').jntadr[0]
    dof = sim.model.jnt_dofadr[j]
    assert np.linalg.norm(sim.data.qvel[dof:dof+6]) < 0.02
    report = {'success': pilot.success, 'controller': 'feedback_rules', 'fresh_ai_inference': False,
              'simulation_seconds': round(sim.data.time, 2), 'shared_top_m': initial_top, 'lift_m': pilot.lift_m,
              'carry_m': pilot.carry_m, 'vehicle_climb_m': pilot.max_rise,
              'ramp_placement_error_m': error, 'final_state': sim.observe()}
    folder = ROOT / '.test-results'
    folder.mkdir(exist_ok=True)
    (folder / 'portable-ramp-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k != 'final_state'}, indent=2))


if __name__ == '__main__':
    main()
