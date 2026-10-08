"""Waypoint feedback control, with contact and timeout failure detection."""
import math
import numpy as np
import mujoco


class Navigator:
    def __init__(self, sim, waypoints, allow_reverse=False):
        self.waypoints = [np.array(p, dtype=float) for p in waypoints]
        self.index = 0
        self.stage = 'DRIVE'
        self.done = self.success = False
        self.reason = ''
        self.elapsed = self.stationary_steps = 0
        self.forward = 0.0
        self.allow_reverse = bool(allow_reverse)
        self.direction = None
        self.initial_boxes = {n: sim.data.body(n).xpos.copy() for n in ('block_1', 'block_2')}

    def stop(self, success, reason):
        self.done, self.success = True, bool(success)
        self.stage = 'COMPLETE' if success else 'STOPPED'
        self.reason = reason
        return (0, 0, 0)

    def action(self, sim):
        if self.done:
            return (0, 0, 0)
        sim.observe()
        self.elapsed += 1
        if self.elapsed > 12000 or not np.isfinite(sim.data.qpos).all():
            return self.stop(False, 'Navigation timed out or state invalid.')
        for n, origin in self.initial_boxes.items():
            if np.linalg.norm(sim.data.body(n).xpos - origin) > 0.015:
                return self.stop(False, 'Course object moved: ' + n)
        # Any vehicle/box contact is a failure, even if the box barely moved.
        box_ids = {sim.model.body(n).id for n in self.initial_boxes}
        for contact in sim.data.contact:
            b1, b2 = (int(sim.model.geom_bodyid[g]) for g in contact.geom)
            if (b1 in box_ids) != (b2 in box_ids):
                other = b2 if b1 in box_ids else b1
                if other != 0:
                    return self.stop(False, 'Vehicle touched a course object.')
        base = sim.data.body('dozer')
        w, x, y, z = base.xquat
        upright = 1 - 2 * (x*x + y*y)
        if upright < 0.7:
            return self.stop(False, 'Vehicle tilted too far.')
        if self.index == len(self.waypoints):
            self.stage = 'SETTLE'
            self.stationary_steps += 1
            if self.stationary_steps > 50:
                distance = float(np.linalg.norm(base.xpos[:2] - self.waypoints[-1]))
                return self.stop(distance < 0.22 and np.linalg.norm(sim.data.qvel[:2]) < 0.03,
                                 f'Final position error: {distance:.3f} m')
            return (0, 0, 0)
        delta = self.waypoints[self.index] - base.xpos[:2]
        distance = np.linalg.norm(delta)
        if distance < 0.14:
            self.index += 1
            self.forward = 0
            self.direction = None
            return (0, 0, 0)
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        heading = math.atan2(delta[1], delta[0])
        error = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
        if self.direction is None:
            # Choose once per waypoint to avoid forward/reverse chatter.
            # A modest bias favors forward travel near sideways targets.
            self.direction = -1 if self.allow_reverse and abs(error) > math.pi/2 + .2 else 1
        if self.direction < 0:
            error = (error + math.pi + math.pi) % (2 * math.pi) - math.pi
        target_speed = min(0.6, distance * 0.9) if abs(error) < 0.25 else 0
        target_speed *= self.direction
        self.forward = float(np.clip(target_speed, self.forward - 0.04, self.forward + 0.025))
        turn = math.copysign(min(0.9, max(0.65, abs(error) * 2)), error) if abs(error) > 0.06 else 0.0
        self.stage = f'{"REVERSE" if self.direction < 0 else "FORWARD"} {self.index + 1}/{len(self.waypoints)}'
        return (self.forward, turn, 0)
