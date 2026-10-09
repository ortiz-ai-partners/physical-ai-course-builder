"""Physical integration checks for the pinned planner, speed gate and failure stops."""
import copy
import json
import xml.etree.ElementTree as ET
from rs_course import *


def main():
    results=[]
    for case in SCENARIOS:
        plan=build_plan(case)
        for high_speed in (False,True):
            result,trace=evaluate(plan,high_speed=high_speed)
            assert result['success'],result
            assert result['curve_peak_km_h']<1.8,result
            assert all(v<.03 for v in result['switch_speeds_m_s']),result
            if high_speed:assert result['peak_km_h']>5.,result
            for row in trace:
                sec=plan['sections'][row[6]]
                if abs(row[4])>1.0001:
                    assert sec['type']=='straight' and sec['direction']==1
            results.append(result)
        assert results[-1]['seconds']<results[-2]['seconds']
        assert results[-1]['direction_changes']==(0 if case=='wide' else 4)
        baseline=results[-1]
        result,trace=evaluate(plan,sport=True)
        assert result['success'] and result['seconds']<baseline['seconds'],result
        assert result['curve_peak_km_h']<3.,result
        assert all(v<.03 for v in result['switch_speeds_m_s']),result
        assert result['direction_changes']==baseline['direction_changes'],result
        for row in trace:
            sec=plan['sections'][row[6]]
            if abs(row[4])>1.0001:
                assert sec['type']=='straight' and sec['direction']==1
        results.append(result)
        print(case,'PASS',flush=True)
    plan=build_plan('wide')
    blocked=copy.deepcopy(plan['config']);blocked['obstacles'].append([-4.2,-3.8,-.8,.8])
    assert not path_clear([p for s in plan['sections'] for p in s['points']],blocked)
    root=ET.fromstring(scene_xml(plan))
    ET.SubElement(root.find('worldbody'),'geom',name='unexpected_obstacle',type='box',
                  pos='-3 0 .4',size='.2 1 .4')
    sim=Simulation(scene_xml=ET.tostring(root,encoding='unicode'));pilot=CoursePilot(sim,plan,sport=True)
    while not pilot.done:
        action=pilot.action(sim)
        if not pilot.done:sim.step(*action)
    assert not pilot.success and 'touched' in pilot.reason,pilot.reason
    try:CoursePilot(sim,plan,empty=False,sport=True);raise AssertionError('Loaded mode accepted')
    except ValueError:pass
    # A body added after planning must not silently enter the high-speed mode.
    body=ET.SubElement(root.find('worldbody'),'body',name='cargo',pos='-5 2 .5')
    ET.SubElement(body,'freejoint');ET.SubElement(body,'geom',type='box',size='.2 .2 .2',mass='1')
    loaded=Simulation(scene_xml=ET.tostring(root,encoding='unicode'))
    try:CoursePilot(loaded,plan,sport=True);raise AssertionError('Extra movable body accepted')
    except ValueError:pass
    report={'trials':results,'footprint_obstacle_rejected':True,'unexpected_contact_stopped':True,
            'loaded_mode_rejected':True,'scope':'Two fixed empty-vehicle courses. Not learned policy, general obstacle replanning, or loaded transport.'}
    dest=ROOT/'.test-results/rs-course-verification.json'
    dest.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
