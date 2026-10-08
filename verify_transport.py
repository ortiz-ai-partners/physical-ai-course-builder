"""Integration checks: real contact transport to several unseen target positions."""
import copy
import json
import numpy as np
from app import Simulation
from layout import load_layout, ROOT
from transport import Transport


def main():
    results = []
    for x, y in [(-1, -3), (0, 0), (1.5, 0.75), (4, 3)]:
        layout = load_layout(ROOT / 'examples' / 'gate.json')
        layout['blocks'][0].update(x=x, y=y)
        sim = Simulation(layout=layout, construction=True)
        pilot = Transport(sim, layout['blocks'][0])
        original = sim.observe()['objects']['block_1']['position']
        while not pilot.done:
            sim.step(*pilot.action(sim))
        final = sim.observe()['objects']['block_1']['position']
        assert pilot.success, (x, y, pilot.reason)
        assert sim.model.neq == 0
        assert np.isfinite(sim.data.qpos).all()
        for _ in range(100):
            sim.step()
        np.testing.assert_allclose(sim.observe()['objects']['block_1']['position'], final, atol=0.005)
        results.append({'target': [x, y], 'success': pilot.success,
                        'error_m': float(np.linalg.norm(np.array(final[:2]) - [x, y])),
                        'lift_m': pilot.max_rise, 'raised_travel_m': pilot.raised_travel,
                        'simulation_seconds': pilot.elapsed * 0.02})
    # A dropped box must stop the controller instead of declaring completion.
    sim = Simulation(layout=layout, construction=True)
    pilot = Transport(sim, layout['blocks'][0])
    pilot.stage = 'CARRY'
    sim.step(*pilot.action(sim))
    assert pilot.done and not pilot.success
    print(json.dumps({'cases': results, 'drop_detection': 'PASS'}, indent=2))


if __name__ == '__main__':
    main()
