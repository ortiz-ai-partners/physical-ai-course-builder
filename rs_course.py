"""Pinned rsplan paths, footprint screening and empty-vehicle physical tracking.

Geometric planning + feedback rules, not a learned driving policy.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import glfw
from app import ROOT, Simulation

sys.path.insert(0, str(ROOT/'third_party/rsplan'))
import rsplan

UPSTREAM = '47b3ab572a4b7ffc314dfc2aa68b8349f061fe13'
SCENARIOS = {
    'wide': {'height': 4., 'radius': 2., 'bounds': [-8, 4, -1.4, 5.4],
             'obstacles': [[-7.5, -.8, 1.4, 2.6]]},
    'narrow': {'height': 1.6, 'radius': 1.5, 'bounds': [-8, 3.2, -1.5, 3.1],
               'obstacles': []},
}


def wrap(x): return (x+math.pi) % (2*math.pi)-math.pi


def overlaps(poly, box):
    rect = np.array([[box[0],box[2]],[box[1],box[2]],[box[1],box[3]],[box[0],box[3]]])
    for edge in list(np.roll(poly,-1,axis=0)-poly)+[np.array([1,0]),np.array([0,1])]:
        axis=np.array([-edge[1],edge[0]])
        a,b=poly@axis,rect@axis
        if max(a)<min(b) or max(b)<min(a): return False
    return True


def footprint(pose, margin=.10):
    # Conservative rectangle includes the fork: local x [-.6, 1.5], y +/-.52.
    x,y,yaw=pose; c,s=math.cos(yaw),math.sin(yaw)
    local=np.array([[-.6-margin,-.52-margin],[1.5+margin,-.52-margin],
                    [1.5+margin,.52+margin],[-.6-margin,.52+margin]])
    return local@np.array([[c,s],[-s,c]])+np.array([x,y])


def path_clear(points, config):
    xmin,xmax,ymin,ymax=config['bounds']
    for p in points:
        poly=footprint(p[:3])
        if poly[:,0].min()<xmin or poly[:,0].max()>xmax or poly[:,1].min()<ymin or poly[:,1].max()>ymax:
            return False
        if any(overlaps(poly,b) for b in config['obstacles']):return False
    return True


def build_plan(case):
    config=SCENARIOS[case]; h=config['height']; radius=config['radius']
    start=(-6.,0.,0.); turn_start=(0.,0.,0.); turn_end=(0.,h,math.pi); end=(-6.,h,math.pi)
    turn=rsplan.path(turn_start,turn_end,radius,0,.04,length_tolerance=0.)
    paths=[rsplan.path(start,turn_start,radius,0,.04,length_tolerance=0.),turn,
           rsplan.path(turn_end,end,radius,0,.04,length_tolerance=0.)]
    sections=[]
    for path in paths:
        pose=path.start_pose
        for seg in path.segments:
            if abs(seg.length)<1e-7:continue
            pts=seg.calc_waypoints(pose,.04,False)
            sections.append({'type':seg.type,'direction':seg.direction,
                             'points':[list(p[:3]) for p in pts], 'length':float(abs(seg.length))})
            pose=tuple(pts[-1][:3])
    all_points=[p for sec in sections for p in sec['points']]
    if not path_clear(all_points,config):raise ValueError('Generated path fails footprint screening; no physical run started')
    return {'schema':'rs-course-v1','case':case,'start':start,'goal':end,'sections':sections,
            'config':config,'upstream_commit':UPSTREAM,'planner':'rsplan / Reeds-Shepp',
            'collision_screen':'sampled footprint, 4cm spacing, 10cm margin; physical contact checks also required',
            'new_ai_inference':False,'learning_performed':False}


def scene_xml(plan):
    root=ET.parse(ROOT/'scene_fork_tracks.xml').getroot();world=root.find('worldbody')
    for b in list(world.findall('body')):
        if b.get('name')!='dozer':world.remove(b)
    for g in list(world.findall('geom')):
        if g.get('name')!='floor':world.remove(g)
    world.find("geom[@name='floor']").set('size','14 10 .1')
    world.find("body[@name='dozer']").set('pos',f'{plan["start"][0]} {plan["start"][1]} .3')
    def box(name,rect):
        a,b,c,d=rect
        ET.SubElement(world,'geom',name=name,type='box',pos=f'{(a+b)/2} {(c+d)/2} .3',
                      size=f'{(b-a)/2} {(d-c)/2} .3',rgba='.2 .4 .45 1')
    a,b,c,d=plan['config']['bounds']
    for i,rect in enumerate(([a-.1,a,c,d],[b,b+.1,c,d],[a,b,c-.1,c],[a,b,d,d+.1])):box('wall_'+str(i),rect)
    for i,rect in enumerate(plan['config']['obstacles']):box('obstacle_'+str(i),rect)
    for sec in plan['sections']:
        for p in sec['points'][::4]:
            ET.SubElement(world,'geom',type='sphere',pos=f'{p[0]} {p[1]} .012',size='.035',
                          rgba='.1 .8 .4 1' if sec['direction']>0 else '1 .35 .2 1',contype='0',conaffinity='0')
    return ET.tostring(root,encoding='unicode')


class CoursePilot:
    def __init__(self,sim,plan,empty=True,high_speed=True):
        if not empty:raise ValueError('This high-speed experiment only accepts an empty vehicle')
        vehicle=sim.model.body('dozer').id
        if any(int(root) not in (0,vehicle) for root in sim.model.body_rootid):
            raise ValueError('Additional movable bodies are outside this empty-vehicle experiment')
        self.plan=plan;self.index=0;self.nearest=0;self.forward=0.;self.integral=0.
        self.high_speed=high_speed
        self.done=self.success=False;self.reason='';self.stage='READY';self.hold=0
        self.peak=0.;self.curve_peak=0.;self.switch_speeds=[];self.max_error=0.;self.traces=[]

    def stop(self,success,reason):
        self.done=True;self.success=bool(success);self.reason=reason;self.stage='SUCCESS' if success else 'FAILED'
        return (0.,0.,0.)

    def action(self,sim):
        if self.done:return (0,0,0)
        sim.observe();base=sim.data.body('dozer');x,y=base.xpos[:2];w,qx,qy,z=base.xquat
        yaw=math.atan2(2*(w*z+qx*qy),1-2*(qy*qy+z*z));speed=float(np.linalg.norm(sim.data.qvel[:2]))
        self.peak=max(self.peak,speed)
        if sim.data.time>240:return self.stop(False,'Time limit')
        if not np.isfinite(sim.data.qpos).all() or 1-2*(qx*qx+qy*qy)<.7:return self.stop(False,'Invalid state or tilt')
        floor=sim.model.geom('floor').id;vehicle=sim.model.body('dozer').id
        for contact in sim.data.contact:
            ga,gb=map(int,contact.geom)
            if floor in (ga,gb):continue
            roots=[int(sim.model.body_rootid[sim.model.geom_bodyid[g]]) for g in (ga,gb)]
            if (roots[0]==vehicle)!=(roots[1]==vehicle):return self.stop(False,'Vehicle/fork touched course geometry')
        sec=self.plan['sections'][self.index];points=np.asarray(sec['points']);pos=np.array([x,y]);direction=sec['direction']
        distances=np.linalg.norm(points[self.nearest:,:2]-pos,axis=1)
        self.nearest+=int(np.argmin(distances));err=float(distances.min());self.max_error=max(self.max_error,err)
        if err>.65:return self.stop(False,'Path deviation exceeded 65 cm')
        endpoint=points[-1];dist=float(np.linalg.norm(endpoint[:2]-pos));angle=wrap(endpoint[2]-yaw)
        if dist<.12 or self.hold:
            self.forward=0.;self.integral=0.;self.hold+=1;self.stage='STOP AT SEGMENT END'
            if abs(angle)>.035:
                self.hold=1
                return (0,math.copysign(.8,angle),0)
            if speed<.03 and abs(sim.data.qvel[5])<.04 and self.hold>30:
                if self.index==len(self.plan['sections'])-1:
                    return self.stop(dist<.20 and abs(angle)<math.radians(10),f'Final error {dist:.3f} m, {math.degrees(abs(angle)):.1f} deg')
                next_dir=self.plan['sections'][self.index+1]['direction']
                if next_dir!=direction:self.switch_speeds.append(speed)
                self.index+=1;self.nearest=0;self.hold=0
            return (0,0,0)
        # Small lookahead for curves; longer at cruise. Heading feedback includes reversing.
        look=.3 if sec['type']!='straight' else .65
        target=points[min(len(points)-1,self.nearest+max(1,int(look/.04)))]
        dx,dy=target[:2]-pos;c,s=math.cos(yaw),math.sin(yaw)
        lateral=-s*dx+c*dy
        remaining=(len(points)-1-self.nearest)*.04
        heading=wrap(points[self.nearest,2]-yaw)
        fast=sec['type']=='straight' and direction>0 and remaining>2 and abs(heading)<.08 and err<.12
        target_f=(1.6 if self.high_speed else 1.0) if fast else (.55 if sec['type']=='straight' else .20)
        target_f=min(target_f,max(.06,dist*.65))*direction
        self.forward=float(np.clip(target_f,self.forward-.035,self.forward+.035))
        desired_yaw=2*(self.forward*.89)*lateral/max(dx*dx+dy*dy,.06)+.6*heading
        desired_yaw=float(np.clip(desired_yaw,-.32,.32))
        yaw_error=desired_yaw-float(sim.data.qvel[5]);self.integral=float(np.clip(self.integral+yaw_error*.02,-.6,.6))
        turn=float(np.clip(3*yaw_error+2*self.integral,-1,1))
        self.stage=('FAST STRAIGHT' if fast else 'CURVE' if sec['type']!='straight' else 'APPROACH')+(' / REVERSE' if direction<0 else ' / FORWARD')
        if sec['type']!='straight':self.curve_peak=max(self.curve_peak,speed)
        self.traces.append([round(float(sim.data.time),3),float(x),float(y),speed,self.forward,turn,self.index])
        return (self.forward,turn,0)


def evaluate(plan,high_speed=True):
    sim=Simulation(scene_xml=scene_xml(plan));pilot=CoursePilot(sim,plan,high_speed=high_speed)
    while not pilot.done:
        before=sim.data.qpos.copy();action=pilot.action(sim)
        assert np.array_equal(before,sim.data.qpos),'Controller directly modified positions'
        if not pilot.done:sim.step(*action)
    return {'case':plan['case'],'success':pilot.success,'reason':pilot.reason,'seconds':float(sim.data.time),
            'peak_km_h':pilot.peak*3.6,'curve_peak_km_h':pilot.curve_peak*3.6,
            'direction_changes':len(pilot.switch_speeds),'switch_speeds_m_s':pilot.switch_speeds,
            'max_path_error_m':pilot.max_error,'sections':[(s['type'],s['direction']) for s in plan['sections']],
            'planner_commit':UPSTREAM,'high_speed_straights':high_speed,
            'control':'feedback rules; empty only','learning_performed':False},pilot.traces


def view(plan,screenshot=None):
    if not glfw.init():raise RuntimeError('GLFW unavailable')
    window=None;context=None
    try:
        if screenshot:glfw.window_hint(glfw.VISIBLE,glfw.FALSE)
        window=glfw.create_window(1280,800,'rsplan | '+plan['case']+' hairpin | physical tracking',None,None)
        if not window:raise RuntimeError('Window unavailable')
        glfw.make_context_current(window);glfw.swap_interval(1)
        sim=Simulation(scene_xml=scene_xml(plan));pilot=CoursePilot(sim,plan)
        if screenshot:
            while not pilot.done:
                action=pilot.action(sim)
                if not pilot.done:sim.step(*action)
        camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
        camera.lookat[:]=[-2,plan['config']['height']/2,0];camera.distance=15;camera.azimuth=90;camera.elevation=-70
        scene=mujoco.MjvScene(sim.model,maxgeom=4000);option=mujoco.MjvOption()
        context=mujoco.MjrContext(sim.model,mujoco.mjtFontScale.mjFONTSCALE_150)
        running=False
        def key(w,k,sc,a,m):
            nonlocal running,pilot
            if a!=glfw.PRESS:return
            if k==glfw.KEY_SPACE:running=not running
            if k==glfw.KEY_R:sim.reset();pilot=CoursePilot(sim,plan);running=False
        glfw.set_key_callback(window,key);last=time.perf_counter();acc=0
        while not glfw.window_should_close(window):
            glfw.poll_events()
            if glfw.get_key(window,glfw.KEY_ESCAPE)==glfw.PRESS:break
            now=time.perf_counter();acc+=min(now-last,.1);last=now
            while acc>=.02:
                if running and not pilot.done:
                    action=pilot.action(sim)
                    if not pilot.done:sim.step(*action)
                acc-=.02
            sim.update_tracks();width,height=glfw.get_framebuffer_size(window)
            if not width or not height:time.sleep(.02);continue
            viewport=mujoco.MjrRect(0,0,width,height)
            mujoco.mjv_updateScene(sim.model,sim.data,option,None,camera,mujoco.mjtCatBit.mjCAT_ALL,scene)
            mujoco.mjr_render(viewport,scene,context)
            text=f'RSPLAN / {plan["case"].upper()} HAIRPIN\nGeometric planning + rule control / NO LEARNING\nSpace: start/pause | R: reset | Esc: close\nGreen: forward path | Orange: reverse path\n{pilot.stage} {"(PAUSED)" if not running else ""}\nSpeed: {np.linalg.norm(sim.data.qvel[:2])*3.6:.2f} km/h | Peak: {pilot.peak*3.6:.2f}\n{pilot.reason}'
            mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,viewport,text,'',context)
            if screenshot:
                from PIL import Image
                pixels=np.empty((height,width,3),dtype=np.uint8)
                mujoco.mjr_readPixels(pixels,None,viewport,context)
                Image.fromarray(np.flipud(pixels)).save(screenshot)
                return pilot.success
            glfw.swap_buffers(window)
    finally:
        if context:context.free()
        if window:glfw.destroy_window(window)
        glfw.terminate()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--case',choices=SCENARIOS,default='wide')
    parser.add_argument('--report',type=Path);parser.add_argument('--screenshot',type=Path)
    args=parser.parse_args();plan=build_plan(args.case)
    if args.report:
        if args.report.exists():parser.error('Choose a new report filename')
        report,traces=evaluate(plan);args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps({'plan':plan,'result':report,'trace':traces},indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2));raise SystemExit(0 if report['success'] else 1)
    view(plan,args.screenshot)
