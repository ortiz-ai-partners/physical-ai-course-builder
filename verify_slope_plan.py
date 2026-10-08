"""Reject unsupported AI plans; verify plan changes reach the physical world."""
import copy
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
from slope_plan import load_plan, validate_plan, evaluate, CAPABILITIES

ROOT = Path(__file__).resolve().parent


def main():
    plan = load_plan(ROOT/'examples/ortiz-slope-plan.json')
    folder = ROOT/'.test-results'; folder.mkdir(exist_ok=True)
    rejected = []
    mutations = [
        ('unknown executable field', lambda p: p.update(command='anything')),
        ('reversed build order', lambda p: p.update(construction_order=['down_1', 'up_1'])),
        ('reversed crossing', lambda p: p.update(crossing_direction='negative_y')),
        ('misaligned connection', lambda p: p['parts'][1].update(x=2)),
        ('overlap', lambda p: p['parts'][1].update(y=0)),
        ('unsupported rotation', lambda p: p['parts'][0].update(yaw=90)),
        ('unsupported location', lambda p: p['parts'][0].update(x=4)),
        ('boolean coordinate', lambda p: p['parts'][0].update(x=True)),
        ('nonfinite coordinate', lambda p: p['parts'][0].update(x=float('nan'))),
        ('duplicate part', lambda p: p['parts'][1].update(id='up_1')),
        ('missing part', lambda p: p['parts'].pop()),
    ]
    for label, mutate in mutations:
        bad = copy.deepcopy(plan); mutate(bad)
        path = folder/'invalid-slope-plan.json'
        path.write_text(json.dumps(bad), encoding='utf-8')
        with patch('app.Simulation', side_effect=AssertionError('Rejected plan started physics')):
            report = evaluate(path)
        assert not report['success'] and not report['construction_performed']
        assert report['phase'] == 'validation_rejected' and report['reason']
        rejected.append(label)
    reports = []
    for x in CAPABILITIES['target_x_choices_m']:
        candidate = copy.deepcopy(plan)
        for part in candidate['parts']: part['x'] = x
        path = folder/f'slope-plan-x{x}.json'
        path.write_text(json.dumps(candidate, ensure_ascii=False), encoding='utf-8')
        report = evaluate(path)
        (folder/f'slope-plan-x{x}-report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        assert report['success'], (x, report['reason'])
        assert report['plan_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
        assert max(report['placement_errors_m'].values()) < .06
        assert report['vehicle_rise_m'] > .42 and report['contacted_surface_count'] == 4
        assert not report['fresh_ai_inference'] and report['construction_performed']
        assert all(p['carry_m'] > x + 1.4 for p in report['transported_parts'])
        reports.append({'target_x': x, 'seconds': report['simulation_seconds'],
                        'errors_m': report['placement_errors_m']})
    summary = {'cases': reports, 'rejected_before_physics': rejected}
    (folder/'slope-plan-verification.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__': main()
