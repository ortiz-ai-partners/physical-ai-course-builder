"""Validated file exchange: chat AI plans, rule controller executes, JSON returns.

No API call, code execution from plans, model training, or fresh replay inference.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

CAPABILITIES = {
    'schema': 'slope-course-v1',
    'target_x_choices_m': [1.0, 1.5, 2.0],
    'parts': [
        {'id': 'up_1', 'kind': 'up', 'y': -1.0, 'yaw': 0},
        {'id': 'down_1', 'kind': 'down', 'y': 1.05, 'yaw': 0}],
    'top_height_m': 0.44, 'nominal_joint_gap_m': 0.05,
    'construction_order': ['up_1', 'down_1'],
    'crossing_direction': 'positive_y',
    'initial_storage': 'x=-1.5, same y as targets; vehicle=(-3,-0.75)',
    'limitations': ['Rule control; no loaded turns; fixed staging lanes',
                    'Geometry/mass provisional; simulation only',
                    'Validation checks supported template, not general collision planning'],
}


def exact_keys(value, keys, label):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f'{label}: expected fields {sorted(keys)}')


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_plan(value):
    exact_keys(value, ['schema', 'name', 'rationale', 'parts', 'construction_order',
                       'crossing_direction', 'provenance'], 'plan')
    if value['schema'] != CAPABILITIES['schema']:
        raise ValueError('Unsupported plan schema')
    for key, limit in [('name', 100), ('rationale', 3000)]:
        if not isinstance(value[key], str) or not value[key].strip() or len(value[key]) > limit:
            raise ValueError(f'{key}: nonempty text required (max {limit})')
    parts = value['parts']
    if not isinstance(parts, list) or len(parts) != 2:
        raise ValueError('Exactly one up and one down part required')
    for part, expected in zip(parts, CAPABILITIES['parts']):
        exact_keys(part, ['id', 'kind', 'x', 'y', 'yaw'], 'part')
        if any(not number(part[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('Part coordinates must be finite numbers')
        if any(part[k] != expected[k] for k in expected):
            raise ValueError('Unsupported part/orientation/joint: up y=-1, down y=1.05, yaw=0 required')
        if part['x'] not in CAPABILITIES['target_x_choices_m']:
            raise ValueError('Unsupported target x: choose 1.0, 1.5, or 2.0 m')
    if parts[0]['x'] != parts[1]['x']:
        raise ValueError('Both slopes must share x to form a crossing')
    if value['construction_order'] != CAPABILITIES['construction_order']:
        raise ValueError('Only up_1 then down_1 construction is supported')
    if value['crossing_direction'] != 'positive_y':
        raise ValueError('Only positive_y crossing is supported')
    provenance = value['provenance']
    exact_keys(provenance, ['source', 'note'], 'provenance')
    if any(not isinstance(v, str) or not v.strip() or len(v) > 1000 for v in provenance.values()):
        raise ValueError('Provenance must contain short text')
    # Normalize via JSON to detach caller-owned mutable objects.
    return json.loads(json.dumps(value, allow_nan=False))


def load_plan(path):
    return validate_plan(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def evaluate(path):
    raw = Path(path).read_bytes()
    report = {'schema': 1, 'plan_sha256': hashlib.sha256(raw).hexdigest(),
              'fresh_ai_inference': False, 'controller': 'feedback_rules',
              'construction_performed': False, 'success': False}
    try:
        plan = validate_plan(json.loads(raw.decode('utf-8-sig')))
    except (ValueError, UnicodeError) as exc:
        return dict(report, phase='validation_rejected', reason=str(exc))
    from app import Simulation, ROOT
    from slope_assembly import assembly_xml, SlopeAssembly
    import numpy as np
    sim = Simulation(scene_xml=assembly_xml(ROOT/'scene_fork_tracks.xml'))
    pilot = SlopeAssembly(sim, target_x=plan['parts'][0]['x'])
    while not pilot.done:
        sim.step(*pilot.action(sim))
    errors = {part['id']: float(np.linalg.norm(
        sim.data.body(part['id']).xpos[:2]-[part['x'], part['y']])) for part in plan['parts']}
    return dict(report, phase='assemble_and_cross', plan_name=plan['name'],
                accepted_plan=plan, construction_performed=True, success=pilot.success,
                reason=pilot.reason, simulation_seconds=round(sim.data.time, 2),
                placement_errors_m=errors, transported_parts=pilot.results,
                contacted_surface_count=len(pilot.touched), vehicle_rise_m=pilot.max_rise,
                final_state=sim.observe(),
                note='AI authored a saved plan in chat; replay performs no new inference. Construction performed means attempted; inspect success.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path, nargs='?')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--capabilities', action='store_true')
    args = parser.parse_args()
    if args.capabilities:
        print(json.dumps(CAPABILITIES, ensure_ascii=False, indent=2))
        return 0
    if args.plan is None or args.report is None:
        parser.error('plan and --report required')
    if args.plan.resolve() == args.report.resolve():
        parser.error('Report must not overwrite the plan')
    report = evaluate(args.plan)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('final_state', 'accepted_plan')}, ensure_ascii=False, indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
