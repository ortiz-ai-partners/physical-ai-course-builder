"""Feedback-controlled single-box straight transport; no learned policy.

Only actuator commands are issued. Initial lane alignment is scene setup,
not navigation. Success requires placement, rest, and fork clearance.
"""
import numpy as np


class Transport:
    def __init__(self, sim, target):
        self.target = np.array([target['x'], target['y']], dtype=float)
        self.stage = 'INSERT'
        self.elapsed = 0
        self.stage_steps = 0
        self.done = False
        self.success = False
        self.reason = ''
        self.floor_z = float(sim.data.body('block_1').xpos[2])
        self.max_rise = 0.0
        self.raised_travel = 0.0
        self.lift_position = None

    def change(self, stage):
        self.stage = stage
        self.stage_steps = 0

    def fail(self, reason):
        self.done = True
        self.stage = 'STOPPED'
        self.reason = reason

    def action(self, sim):
        if self.done:
            return (0, 0, 0)
        sim.observe()
        self.elapsed += 1
        self.stage_steps += 1
        box = sim.data.body('block_1').xpos.copy()
        base = sim.data.body('dozer').xpos.copy()
        rise = float(box[2] - self.floor_z)
        self.max_rise = max(self.max_rise, rise)
        if not np.isfinite(sim.data.qpos).all() or self.elapsed > 6000 or self.stage_steps > 3000:
            self.fail('Invalid state or time limit. No completion claimed.')
            return (0, 0, 0)
        if abs(box[1] - self.target[1]) > 0.12:
            self.fail('Box left the straight transport lane.')
            return (0, 0, 0)
        if self.stage == 'INSERT':
            if box[0] - base[0] <= 1.00:
                self.change('LIFT')
            else:
                return (0.25, 0, 0)
        if self.stage == 'LIFT':
            if sim.lift_target < 0.25:
                return (0, 0, 1)
            if self.stage_steps > 100:
                if rise < 0.15:
                    self.fail('The fork did not lift the box.')
                else:
                    self.lift_position = box.copy()
                    self.change('CARRY')
            return (0, 0, 0)
        if self.stage == 'CARRY':
            if rise < 0.12:
                self.fail('Box dropped during transport.')
                return (0, 0, 0)
            self.raised_travel = float(np.linalg.norm(box[:2] - self.lift_position[:2]))
            error = self.target[0] - box[0]
            if abs(error) < 0.015 and abs(sim.data.qvel[0]) < 0.025:
                self.change('LOWER')
                return (0, 0, 0)
            return (float(np.clip(error * 1.2, -0.25, 0.25)), 0, 0)
        if self.stage == 'LOWER':
            if sim.lift_target > 0:
                return (0, 0, -1)
            if self.stage_steps > 120:
                self.change('WITHDRAW')
            return (0, 0, 0)
        if self.stage == 'WITHDRAW':
            if box[0] - base[0] > 1.72:
                self.change('CHECK')
            else:
                return (-0.25, 0, 0)
        if self.stage == 'CHECK' and self.stage_steps > 60:
            joint = sim.model.body('block_1').jntadr[0]
            dof = sim.model.jnt_dofadr[joint]
            speed = float(np.linalg.norm(sim.data.qvel[dof:dof+3]))
            error = float(np.linalg.norm(box[:2] - self.target))
            self.success = (error < 0.06 and abs(rise) < 0.02 and speed < 0.02
                            and box[0] - base[0] > 1.65 and self.max_rise > 0.15
                            and self.raised_travel > 0.3)
            self.done = True
            self.stage = 'COMPLETE' if self.success else 'STOPPED'
            self.reason = f'Position error: {error:.3f} m; box speed: {speed:.3f} m/s'
        return (0, 0, 0)
