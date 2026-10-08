"""File bridge for plans authored by the AI in the interactive Codex chat.

No hidden API or automatic chat polling. A saved plan is a planning snapshot,
not fresh inference. The executor validates a small allowlisted data format.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from layout import validate_layout


def validate_plan(value):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('Plan schema must be 1.')
    name, rationale = value.get('name'), value.get('rationale')
    if not isinstance(name, str) or not 1 <= len(name) <= 100:
        raise ValueError('Plan must have a short name.')
    if not isinstance(rationale, str) or not 1 <= len(rationale) <= 3000:
        raise ValueError('Plan must explain its design choices.')
    layout = validate_layout(value.get('layout'))
    if layout.get('ramps') or layout.get('slopes'):
        raise ValueError('Ramp layouts support preview only; use --ramp for the independent crossing demo.')
    points = value.get('waypoints')
    if not isinstance(points, list) or not 2 <= len(points) <= 20:
        raise ValueError('Plan must contain 2 to 20 waypoints.')
    for p in points:
        if not isinstance(p, list) or len(p) != 2 or any(type(n) not in (int, float) or not math.isfinite(n) for n in p):
            raise ValueError('Each waypoint must have two finite numbers.')
        # Conservative envelope for the vehicle including its forward forks.
        if not (-4.2 <= p[0] <= 4.2 and -2.7 <= p[1] <= 2.7):
            raise ValueError('Waypoint too close to arena wall.')
    return {'schema': 1, 'name': name, 'rationale': rationale,
            'layout': layout, 'waypoints': points,
            'provenance': value.get('provenance', {'source': 'unspecified'})}


def load_plan(path):
    return validate_plan(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def evaluate(path, assemble=False):
    from app import Simulation
    from navigation import Navigator
    plan = load_plan(path)
    if assemble:
        from assembly import Assembly, validate_gate
        validate_gate(plan)
    sim = Simulation(layout=plan['layout'], construction='gate' if assemble else False)
    pilot = Assembly(sim, plan) if assemble else Navigator(sim, plan['waypoints'])
    while not pilot.done:
        sim.step(*pilot.action(sim))
    return {'schema': 1, 'plan_sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
            'plan_name': plan['name'], 'phase': 'assemble_and_drive' if assemble else 'drive_existing_course',
            'construction_performed': assemble, 'placed_boxes': pilot.results if assemble else [], 'fresh_ai_inference': False,
            'controller': 'waypoint_feedback_rules', 'success': pilot.success,
            'reason': pilot.reason, 'visited_waypoints': getattr(pilot.pilot, 'index', 0) if assemble else pilot.index,
            'simulation_seconds': pilot.elapsed * 0.02,
            'final_state': sim.observe(),
            'note': 'The AI authored the plan in chat. This execution evaluates its saved snapshot.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('plan', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--assemble', action='store_true')
    args = parser.parse_args()
    report = evaluate(args.plan, args.assemble)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'final_state'}, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report['success'] else 1)
