"""Restricted two-lane gate assembly and driving in ONE persistent simulation."""
import math
import numpy as np
from transport import Transport
from navigation import Navigator


def validate_gate(plan):
    a, b = plan['layout']['blocks']
    if not (0.5 <= a['x'] <= 3 and a['x'] == b['x']
            and 0.75 <= a['y'] <= 1.5 and -1.5 <= b['y'] <= -0.75):
        raise ValueError('Assembly requires equal X in [0.5,3], red Y in [0.75,1.5], blue Y in [-1.5,-0.75].')


class Assembly:
    def __init__(self, sim, plan):
        validate_gate(plan)
        self.plan = plan
        self.phase = 'RED'
        self.stage = self.phase
        self.done = self.success = False
        self.reason = ''
        self.elapsed = 0
        self.pilot = Transport(sim, plan['layout']['blocks'][0])
        self.results = []
        self.placed = {}

    def angle(self, sim):
        w,x,y,z=sim.data.body('dozer').xquat
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

    def rotate(self, sim, goal):
        error=(goal-self.angle(sim)+math.pi)%(2*math.pi)-math.pi
        if abs(error)<0.009:
            return None
        return (0,math.copysign(min(0.9,max(0.65,abs(error)*2)),error),0)

    def action(self, sim):
        if self.done:
            return (0,0,0)
        sim.observe()
        self.elapsed += 1
        if self.elapsed > 30000:
            self.done=True; self.stage='STOPPED'; self.reason='Assembly time limit'
            return (0,0,0)
        for name,position in self.placed.items():
            if np.linalg.norm(sim.data.body(name).xpos-position)>0.025:
                self.done=True; self.stage='STOPPED'; self.reason='Placed box disturbed: '+name
                return (0,0,0)
        self.stage=self.phase
        base=sim.data.body('dozer').xpos
        if self.phase in ('RED','BLUE','DRIVE'):
            action=self.pilot.action(sim)
            self.stage=self.phase+': '+self.pilot.stage
            if self.pilot.done:
                if not self.pilot.success:
                    self.done=True; self.reason=self.pilot.reason
                elif self.phase=='DRIVE':
                    self.done=self.success=True; self.stage='COMPLETE'; self.reason=self.pilot.reason
                else:
                    name='block_1' if self.phase=='RED' else 'block_2'
                    self.placed[name]=sim.data.body(name).xpos.copy()
                    self.results.append({'box':name,'position':self.placed[name].tolist(),
                                         'lift_m':self.pilot.max_rise,'transport_m':self.pilot.raised_travel})
                    self.phase='RETREAT_RED' if self.phase=='RED' else 'RETREAT_BLUE'
            return action
        if self.phase.startswith('RETREAT'):
            if base[0]>-3.3:
                return (-min(0.6,max(0.10,(base[0]+3.3)*1.5)),0,0)
            self.phase='TURN_DOWN' if self.phase=='RETREAT_RED' else 'TURN_UP'
            return (0,0,0)
        if self.phase in ('TURN_DOWN','TURN_UP','ALIGN_BLUE','ALIGN_DRIVE'):
            heading={'TURN_DOWN':-math.pi/2,'TURN_UP':math.pi/2,'ALIGN_BLUE':0,'ALIGN_DRIVE':0}[self.phase]
            action=self.rotate(sim,heading)
            if action is not None:
                return action
            if self.phase=='TURN_DOWN': self.phase='TO_BLUE'
            elif self.phase=='TURN_UP': self.phase='TO_CENTER'
            elif self.phase=='ALIGN_BLUE':
                self.phase='BLUE'; self.pilot=Transport(sim,self.plan['layout']['blocks'][1],'block_2')
            else:
                self.phase='DRIVE'; self.pilot=Navigator(sim,self.plan['waypoints'])
            return (0,0,0)
        if self.phase in ('TO_BLUE','TO_CENTER'):
            target=self.plan['layout']['blocks'][1]['y'] if self.phase=='TO_BLUE' else 0
            error=target-base[1]
            if abs(error)<0.012:
                self.phase='ALIGN_BLUE' if self.phase=='TO_BLUE' else 'ALIGN_DRIVE'
                return (0,0,0)
            heading=-math.pi/2 if self.phase=='TO_BLUE' else math.pi/2
            correction=self.rotate(sim,heading)
            if correction is not None: return correction
            sign=-1 if self.phase=='TO_BLUE' else 1
            return (float(np.clip(error*sign*1.5,-0.35,0.35)),0,0)
        raise RuntimeError('Unknown assembly state')
