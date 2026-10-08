"""Construct a two-piece bridge and cross it without resetting physics."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from slope_parts import add_slope
from transport import Transport
from navigation import Navigator

# Calibrated staging estimate for this scene, not a general vehicle model.
# Actual heading is checked again by Transport before lifting the second part.
TURN_LATERAL_ESTIMATE = 0.164
INSTALLATION_GAP = 0.05


def assembly_xml(scene_path):
    root=ET.parse(scene_path).getroot()
    world=root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') in ('block_3','block_4','ball'):
            world.remove(body)
    for name,y in [('block_1',3),('block_2',-3)]:
        world.find(f"body[@name='{name}']").set('pos',f'4 {y} 0.26')
    world.find("body[@name='dozer']").set('pos','-3 -0.75 0.30')
    for kind,y in [('up',-1),('down',1+INSTALLATION_GAP)]:
        add_slope(root,{'id':kind+'_1','kind':kind,'x':-1.5,'y':y,'yaw':0})
    return ET.tostring(root,encoding='unicode')


class SlopeAssembly:
    def __init__(self,sim):
        self.phase='UP'
        self.stage=self.phase
        self.reason=''
        self.done=self.success=False
        self.elapsed=0
        self.placed={}
        self.results=[]
        self.max_rise=0.0
        self.touched=set()
        self.pilot=Transport(sim,{'x':1.5,'y':-1},'up_1',1.28,2.05,lane_y=-.75)
        self.surfaces={sim.model.geom(n+s).id for n in ('up_1','down_1') for s in ('_incline','_landing')}
        self.wheels={sim.model.body(n).id for n in ('left_front','left_rear','right_front','right_rear')}

    def stop(self,reason):
        self.done=True
        self.stage='COMPLETE' if self.success else 'STOPPED'
        self.reason=reason
        return (0,0,0)

    def action(self,sim):
        if self.done:return (0,0,0)
        sim.observe()
        self.elapsed+=1
        self.stage=self.phase
        if self.elapsed>25000 or not np.isfinite(sim.data.qpos).all():
            return self.stop('Time limit or invalid state')
        for name,pos in self.placed.items():
            if np.linalg.norm(sim.data.body(name).xpos-pos)>.04:
                return self.stop('Placed slope disturbed: '+name)
        if self.phase in ('UP','DOWN','TO_DOWN','TO_START','CROSS'):
            action=self.pilot.action(sim)
            self.stage=self.phase+': '+self.pilot.stage
            if self.phase=='CROSS':
                self.max_rise=max(self.max_rise,float(sim.data.body('dozer').xpos[2])-.3)
                for c in sim.data.contact:
                    a,b=map(int,c.geom)
                    for ground,wheel in ((a,b),(b,a)):
                        if ground in self.surfaces and int(sim.model.geom_bodyid[wheel]) in self.wheels:
                            self.touched.add(ground)
            if self.pilot.done:
                if not self.pilot.success:return self.stop(self.pilot.reason)
                if self.phase in ('UP','DOWN'):
                    name='up_1' if self.phase=='UP' else 'down_1'
                    self.placed[name]=sim.data.body(name).xpos.copy()
                    self.results.append({'part':name,'lift_m':self.pilot.max_rise,
                                         'carry_m':self.pilot.raised_travel})
                    self.phase='RAISE_UP' if self.phase=='UP' else 'RAISE_DOWN'
                elif self.phase=='TO_DOWN':self.phase='PRECISE_LANE'
                elif self.phase=='TO_START':
                    self.phase='CROSS';self.pilot=Navigator(sim,[[1.5,3.2]])
                else:
                    self.success=self.max_rise>.42 and self.touched==self.surfaces
                    return self.stop(f'Placed 2 slopes; crossed {len(self.touched)}/4 surfaces; rise {self.max_rise:.3f} m')
            return action
        if self.phase.startswith('RAISE'):
            if sim.lift_target<.49:return (0,0,1)
            if self.phase=='RAISE_UP':
                self.phase='TO_DOWN';self.pilot=Navigator(sim,[[-3.3,-.75],[-3.3,.8]])
            else:
                self.phase='TO_START';self.pilot=Navigator(sim,[[-1,.75],[-1,-3.2],[1.5,-3.2]])
            return (0,0,0)
        if self.phase=='PRECISE_LANE':
            w,x,y,z=sim.data.body('dozer').xquat
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            angle=(math.pi/2-yaw+math.pi)%(2*math.pi)-math.pi
            if abs(angle)>.009:return (0,math.copysign(min(.9,max(.65,abs(angle)*2)),angle),0)
            error=(.8-TURN_LATERAL_ESTIMATE)-sim.data.body('dozer').xpos[1]
            if abs(error)>.003:return (float(np.clip(error*1.5,-.15,.15)),0,0)
            self.phase='ALIGN'
        if self.phase=='ALIGN':
            w,x,y,z=sim.data.body('dozer').xquat
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            if abs(yaw)>.009:return (0,-math.copysign(min(.9,max(.65,abs(yaw)*2)),yaw),0)
            self.phase='RETREAT_FOR_ALIGNMENT'
        if self.phase=='RETREAT_FOR_ALIGNMENT':
            if sim.data.body('dozer').xpos[0]>-4.5:return (-.35,0,0)
            self.phase='LOWER'
        if self.phase=='LOWER':
            if sim.lift_target>0:return (0,0,-1)
            self.phase='DOWN'
            self.pilot=Transport(sim,{'x':1.5,'y':1.05},'down_1',1.28,2.05,lane_y=.8,align_before_lift=True)
            return (0,0,0)
        raise RuntimeError(self.phase)
