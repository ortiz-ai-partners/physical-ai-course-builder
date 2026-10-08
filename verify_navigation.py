"""Navigation success, too-narrow gate rejection, and malformed plan checks."""
import copy
import json
from app import Simulation
from ai_plan import load_plan, validate_plan
from layout import ROOT
from navigation import Navigator


def run(plan):
    sim = Simulation(layout=plan['layout'])
    pilot = Navigator(sim, plan['waypoints'])
    while not pilot.done:
        sim.step(*pilot.action(sim))
    return {'success': pilot.success, 'reason': pilot.reason,
            'visited': pilot.index, 'seconds': pilot.elapsed * 0.02}


def main():
    plan = load_plan(ROOT / 'examples' / 'ortiz-gate-plan.json')
    success = run(plan)
    assert success['success'] and success['visited'] == 4, success
    narrow = copy.deepcopy(plan)
    narrow['layout']['blocks'][0]['y'] = 0.5
    narrow['layout']['blocks'][1]['y'] = -0.5
    failure = run(narrow)
    assert not failure['success'] and 'touched' in failure['reason'], failure
    invalid = copy.deepcopy(plan)
    invalid['waypoints'][0] = [float('nan'), 0]
    try:
        validate_plan(invalid)
        raise AssertionError('Nonfinite waypoint accepted')
    except ValueError:
        pass
    print(json.dumps({'normal_gate': success, 'too_narrow_gate': failure}, indent=2))


if __name__ == '__main__':
    main()
