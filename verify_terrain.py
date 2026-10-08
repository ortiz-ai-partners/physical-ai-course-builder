"""Check real ramp contact and ascent, without resets or pose assignment."""
import json
import numpy as np
from app import Simulation
from terrain import RampPilot


def check(height):
    sim = Simulation(ramp_height=height)
    pilot = RampPilot(sim)
    def forbidden():
        raise AssertionError('Reset during ramp crossing')
    sim.reset = forbidden
    while not pilot.done:
        before = sim.data.qpos.copy()
        action = pilot.action(sim)
        assert np.array_equal(before, sim.data.qpos), 'Controller changed pose directly'
        time_before = sim.data.time
        sim.step(*action)
        assert sim.data.time > time_before
    sim.observe()
    result = dict(height=height, success=pilot.success, reason=pilot.reason,
                  seconds=round(sim.data.time, 2), max_rise=pilot.max_rise,
                  final_position=sim.data.body('dozer').xpos.tolist())
    assert pilot.success, result
    assert abs(sim.data.body('dozer').xpos[2] - pilot.initial_z) < 0.02, result
    return result


if __name__ == '__main__':
    results = [check(h) for h in (0.25, 0.35, 0.45)]
    # A visible ramp without collision must never count as a crossing success.
    sim = Simulation(ramp_height=0.35)
    pilot = RampPilot(sim)
    for g in pilot.terrain_ids:
        sim.model.geom_contype[g] = 0
        sim.model.geom_conaffinity[g] = 0
    while not pilot.done:
        sim.step(*pilot.action(sim))
    assert not pilot.success and not pilot.wheel_contacts and pilot.max_rise < 0.01
    print(json.dumps({'crossings': results, 'visual_only_rejected': True}, indent=2))
