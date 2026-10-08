"""Matched trials: forward-only vs direction choice, plus blocked rear."""
import json
import math
from pathlib import Path
from maneuver import evaluate


def main():
    trials = []
    for target, yaw in [((1, 0), 0), ((-1, 0), 0), ((-1, .5), 0),
                        ((-1, -.5), 0), ((0, -1), math.pi/2)]:
        pair = [evaluate(target, reverse, yaw) for reverse in (False, True)]
        assert all(p['success'] for p in pair), pair
        assert pair[0]['reverse_steps'] == 0
        if target != (1, 0):
            assert pair[1]['reverse_steps'] > 0, pair
        else:
            assert pair[1]['reverse_steps'] == 0
        trials.append(pair)
    straight_back = trials[1]
    assert straight_back[1]['seconds'] < straight_back[0]['seconds']
    assert straight_back[1]['turn_degrees'] < straight_back[0]['turn_degrees']
    blocked = evaluate((-1, 0), True, blocked=True)
    assert not blocked['success'] and ('touched' in blocked['reason'] or 'moved' in blocked['reason'])
    report = {'paired_trials': trials, 'blocked_rear': blocked,
              'limitation': 'Contact causes failure; this is not predictive obstacle avoidance. Empty vehicle and position targets only.'}
    path = Path(__file__).resolve().parent/'.test-results/maneuver-report.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
