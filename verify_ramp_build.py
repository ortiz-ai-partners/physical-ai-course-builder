"""Editor target -> physical construction at multiple positions."""
import copy
import json
import numpy as np
from app import Simulation
from portable_ramp import PortablePilot, validate_ramp_build


def example(x=1.5):
    return {'schema': 1, 'blocks': [{'id': 'block_1', 'x': 4, 'y': 3},
            {'id': 'block_2', 'x': 4, 'y': -3}],
            'ramps': [{'id': 'ramp_1', 'x': x, 'y': 0, 'height': 0.44}]}


def main():
    results = []
    for x in (0.5, 1.25, 2):
        layout = example(x)
        sim = Simulation(layout=layout, portable_ramp=True)
        pilot = PortablePilot(sim, layout)
        assert abs(sim.data.body('portable_ramp').xpos[0] + 1.5) < 0.001
        while not pilot.done:
            before = sim.data.qpos.copy()
            action = pilot.action(sim)
            assert np.array_equal(before, sim.data.qpos)
            sim.step(*action)
        sim.observe()
        error = float(np.linalg.norm(sim.data.body('portable_ramp').xpos[:2] - [x, 0]))
        assert pilot.success and error < 0.06, pilot.reason
        results.append({'target_x': x, 'success': pilot.success, 'error_m': error,
                        'seconds': round(sim.data.time, 2)})
    for key, value in [('x', 2.25), ('y', 0.25), ('height', 0.3)]:
        bad = example()
        bad['ramps'][0][key] = value
        try:
            validate_ramp_build(bad)
        except ValueError:
            continue
        raise AssertionError('Unsupported build accepted')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
