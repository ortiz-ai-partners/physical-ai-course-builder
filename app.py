"""Keyboard teleoperation of a physically simulated four-wheel bulldozer.

No API calls and no learned policy. MuJoCo advances all contact dynamics.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import glfw
import mujoco
import numpy as np
from tracks import TrackAnimation

ROOT = Path(__file__).resolve().parent
OBJECTS = ['dozer', 'blade', 'block_1', 'block_2', 'block_3', 'block_4', 'ball', 'portable_ramp', 'up_1', 'down_1']
CONTROL_DT = 0.02


class Simulation:
    def __init__(self, attachment='fork', layout=None, construction=False, ramp_height=None, portable_ramp=False, scene_xml=None):
        self.attachment = attachment
        self.scene_path = ROOT / ('scene_fork_tracks.xml' if attachment == 'fork' else 'scene_tracks.xml')
        if scene_xml is not None:
            self.model = mujoco.MjModel.from_xml_string(scene_xml)
        elif portable_ramp:
            from portable_ramp import portable_xml
            self.model = mujoco.MjModel.from_xml_string(portable_xml(self.scene_path, layout=layout))
        elif ramp_height is not None:
            from terrain import ramp_xml
            self.model = mujoco.MjModel.from_xml_string(ramp_xml(self.scene_path, ramp_height))
        elif layout is None:
            self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        else:
            from layout import preview_xml
            self.model = mujoco.MjModel.from_xml_string(preview_xml(layout, self.scene_path, construction))
        self.data = mujoco.MjData(self.model)
        self.tracks = TrackAnimation(self.model)
        self.lift_target = 0.0
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.lift_target = 0.0
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)
        # Settled initial state, time starts at zero for the new episode.
        self.data.time = 0.0
        self.update_tracks()

    def update_tracks(self):
        self.tracks.update(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def observe(self):
        # mj_step leaves derived body poses at the previous integration stage.
        # Synchronize them before recording, including between render frames.
        mujoco.mj_forward(self.model, self.data)
        return {
            'time': float(self.data.time),
            'qpos': self.data.qpos.tolist(),
            'qvel': self.data.qvel.tolist(),
            'lift_target': self.lift_target,
            'objects': {name: {'position': self.data.body(name).xpos.tolist(),
                               'quaternion_wxyz': self.data.body(name).xquat.tolist()}
                        for name in OBJECTS if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0},
        }

    def step(self, forward=0.0, turn=0.0, lift=0.0):
        # Positive turn means left. Wheels are speed-controlled, never teleported.
        self.lift_target = float(np.clip(self.lift_target + lift * 0.3 * CONTROL_DT, 0, 0.5))
        left = float(np.clip(forward * 5.0 - turn * 4.0, -8, 8))
        right = float(np.clip(forward * 5.0 + turn * 4.0, -8, 8))
        self.data.ctrl[:] = [left, left, right, right, self.lift_target]
        for _ in range(round(CONTROL_DT / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        return self.data.ctrl.copy()


class Recorder:
    def __init__(self):
        self.file = None
        self.path = None
        self.frames = 0

    def start(self, sim):
        folder = ROOT / 'recordings'
        folder.mkdir(exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path = folder / f'demo_{stamp}.jsonl'
        self.file = self.path.open('x', encoding='utf-8')
        self.frames = 0
        import hashlib
        self.write({'type': 'header', 'schema': 1, 'engine': 'mujoco',
                    'engine_version': mujoco.__version__, 'control_dt': CONTROL_DT,
                    'scene_sha256': hashlib.sha256(sim.scene_path.read_bytes()).hexdigest(),
                    'attachment': sim.attachment,
                    'drive_model': 'four_wheel_contact_proxy_with_animated_tracks',
                    'actions': ['forward', 'turn_left', 'lift_up'],
                    'actuators': ['left_front', 'left_rear', 'right_front', 'right_rear', 'lift_target'],
                    'initial_state': sim.observe(), 'source': 'keyboard_teleoperation',
                    'success': None})

    def write(self, row):
        self.file.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.file.flush()

    def stop(self, reason='user'):
        if self.file:
            self.write({'type': 'end', 'frames': self.frames, 'reason': reason})
            self.file.close()
            self.file = None


def run(screenshot=None, smoke=False, layout=None, construction=False, driving_plan=None, ramp=False, portable_ramp=False, move_slope=None, build_slopes=False):
    if not glfw.init():
        raise RuntimeError('OpenGL window could not initialize.')
    window = None
    context = None
    recorder = Recorder()
    try:
        if screenshot:
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        glfw.window_hint(glfw.SAMPLES, 4)
        title = 'AI plan replay | rule-based driving' if driving_plan else 'Auto Transport | Faster cruise / RED BOX ONLY' if construction else 'Layout Preview | NOT a construction result' if layout else 'Bulldozer Lab | WASD + Space / Shift'
        if ramp:
            title = 'Ramp Crossing | Physical contact / rule control'
        if portable_ramp:
            title = 'Portable Ramp | Lift, place and cross / rule control'
        if move_slope:
            title = 'Single slope transport | '+move_slope
        if build_slopes:
            title = 'Build two slopes and cross | rule control'
        window = glfw.create_window(1280, 800, title, None, None)
        if not window:
            raise RuntimeError('Could not create OpenGL window. Check the graphics driver.')
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        custom_xml = None
        if move_slope:
            from slope_transport import slope_transport_xml
            custom_xml = slope_transport_xml(ROOT/'scene_fork_tracks.xml', move_slope)
        if build_slopes:
            from slope_assembly import assembly_xml
            custom_xml = assembly_xml(ROOT/'scene_fork_tracks.xml')
        sim = Simulation(layout=layout, construction=construction, ramp_height=0.35 if ramp else None, portable_ramp=portable_ramp, scene_xml=custom_xml)
        pilot = None
        if construction == 'gate':
            from assembly import Assembly
            pilot = Assembly(sim, driving_plan)
        elif construction:
            from transport import Transport
            pilot = Transport(sim, layout['blocks'][0])
        if driving_plan and construction != 'gate':
            from navigation import Navigator
            pilot = Navigator(sim, driving_plan['waypoints'])
        if ramp:
            from terrain import RampPilot
            pilot = RampPilot(sim)
        if portable_ramp:
            from portable_ramp import PortablePilot
            pilot = PortablePilot(sim, layout=layout)
        if move_slope:
            from slope_transport import slope_pilot
            pilot = slope_pilot(sim, move_slope)
        if build_slopes:
            from slope_assembly import SlopeAssembly
            pilot = SlopeAssembly(sim)
        if pilot:
            if screenshot:
                while not pilot.done:
                    sim.step(*pilot.action(sim))
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [-0.3, 0, 0.1]
        camera.distance = 11
        camera.azimuth = 140
        camera.elevation = -48
        option = mujoco.MjvOption()
        scene = mujoco.MjvScene(sim.model, maxgeom=2000)
        context = mujoco.MjrContext(sim.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        keys = set()
        ui = {'paused': bool(layout) and not pilot, 'follow': False, 'message': 'Lower forks, slide under a box, then Space to lift. Q = slow drive.',
              'mouse': None}

        def on_key(win, key, scancode, action, mods):
            nonlocal sim, scene, context
            if action == glfw.PRESS:
                if (layout or ramp or portable_ramp or move_slope or build_slopes) and key not in ((glfw.KEY_ESCAPE, glfw.KEY_1, glfw.KEY_2, glfw.KEY_P) if pilot else (glfw.KEY_ESCAPE, glfw.KEY_1, glfw.KEY_2)):
                    return
                keys.add(key)
                if key == glfw.KEY_ESCAPE:
                    glfw.set_window_should_close(win, True)
                elif key == glfw.KEY_R:
                    recorder.stop('reset')
                    sim.reset()
                    keys.clear()
                    ui['message'] = 'Scene reset. Recording closed.'
                elif key == glfw.KEY_P:
                    ui['paused'] = not ui['paused']
                    keys.clear()
                elif key == glfw.KEY_C:
                    ui['follow'] = not ui['follow']
                elif key == glfw.KEY_T:
                    recorder.stop('attachment_changed')
                    sim = Simulation('blade' if sim.attachment == 'fork' else 'fork')
                    scene = mujoco.MjvScene(sim.model, maxgeom=2000)
                    context.free()
                    context = mujoco.MjrContext(sim.model, mujoco.mjtFontScale.mjFONTSCALE_150)
                    keys.clear()
                    ui['message'] = f'Tool changed to {sim.attachment.upper()}. Scene reset; recording closed.'
                elif key == glfw.KEY_F:
                    if recorder.file:
                        recorder.stop()
                        ui['message'] = f'Saved {recorder.frames} steps to recordings/.'
                    else:
                        recorder.start(sim)
                        ui['message'] = 'Recording state / action / next state at 50 Hz.'
                elif key == glfw.KEY_1:
                    camera.azimuth, camera.elevation, camera.distance = 140, -48, 11
                    camera.lookat[:] = [-0.3, 0, 0.1]
                    ui['follow'] = False
                elif key == glfw.KEY_2:
                    camera.azimuth, camera.elevation, camera.distance = 180, -89, 12
                    camera.lookat[:] = [0, 0, 0]
                    ui['follow'] = False
            elif action == glfw.RELEASE:
                keys.discard(key)

        def on_focus(win, focused):
            keys.clear()
            if not focused:
                sim.data.ctrl[:4] = 0

        def on_scroll(win, dx, dy):
            camera.distance = float(np.clip(camera.distance * math.exp(-0.1 * dy), 2, 22))

        def on_cursor(win, x, y):
            old = ui['mouse']
            ui['mouse'] = (x, y)
            if old is None:
                return
            dx, dy = x - old[0], y - old[1]
            if glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
                camera.azimuth -= dx * 0.3
                camera.elevation = float(np.clip(camera.elevation - dy * 0.3, -89, -10))
            elif glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
                width, height = glfw.get_window_size(win)
                mujoco.mjv_moveCamera(sim.model, mujoco.mjtMouse.mjMOUSE_MOVE_H,
                                     dx / max(height, 1), dy / max(height, 1), scene, camera)
                ui['follow'] = False

        glfw.set_key_callback(window, on_key)
        glfw.set_window_focus_callback(window, on_focus)
        glfw.set_scroll_callback(window, on_scroll)
        glfw.set_cursor_pos_callback(window, on_cursor)
        last = time.perf_counter()
        accumulator = 0.0
        started = last
        while not glfw.window_should_close(window):
            glfw.poll_events()
            now = time.perf_counter()
            accumulator = min(accumulator + now - last, 0.1)
            last = now
            focused = glfw.get_window_attrib(window, glfw.FOCUSED)
            while accumulator >= CONTROL_DT:
                accumulator -= CONTROL_DT
                if ui['paused']:
                    continue
                active = keys if focused else set()
                action = [int(glfw.KEY_W in active) - int(glfw.KEY_S in active),
                          int(glfw.KEY_A in active) - int(glfw.KEY_D in active),
                          int(glfw.KEY_SPACE in active) - int(bool({glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT} & active))]
                if glfw.KEY_Q in active:
                    action[0] *= 0.3
                    action[1] *= 0.3
                if pilot:
                    action = pilot.action(sim)
                before = sim.observe() if recorder.file else None
                controls = sim.step(*action)
                if recorder.file:
                    recorder.write({'type': 'transition', 'step': recorder.frames,
                                    'state': before, 'action': action,
                                    'actuator_command': controls.tolist(), 'next_state': sim.observe()})
                    recorder.frames += 1
            if ui['follow']:
                camera.lookat[:] = sim.data.body('dozer').xpos
                camera.distance = min(camera.distance, 6)
            sim.update_tracks()
            width, height = glfw.get_framebuffer_size(window)
            if width == 0 or height == 0:
                time.sleep(0.02)
                continue
            viewport = mujoco.MjrRect(0, 0, width, height)
            mujoco.mjv_updateScene(sim.model, sim.data, option, None, camera,
                                  mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            lift_pos = float(sim.data.joint('lift').qpos[0])
            status = 'PAUSED' if ui['paused'] else ('RECORDING' if recorder.file else 'MANUAL / NO AI')
            left = 'TRACKED DOZER\n\nDrive\nLift\nAttachment\nRecord\nReset / Pause\nCamera\nView\nExit'
            right = f'{status}\n\nW A S D / hold Q: slow\nSpace UP / Shift DOWN\nT: {sim.attachment.upper()} (resets scene)\nF ({recorder.frames} steps)\nR / P\nDrag / wheel / C follow\n1 overview / 2 top\nEsc'
            if layout and not pilot:
                left = 'TARGET LAYOUT PREVIEW\n\nDesired final positions only\nNo autonomous construction / No AI\n\nCamera: drag / wheel\nView: 1 overview / 2 top\nClose: Esc'
                right = ''
                ui['message'] = 'Design preview. Boxes were placed directly for visualization, not moved by the robot.'
            if pilot:
                left = 'AUTO TRANSPORT / RULE CONTROL\n\nRED BOX ONLY / STRAIGHT LANE\nP: pause / resume | Esc: close\nView: 1 / 2 | Mouse: camera'
                if driving_plan:
                    left = 'AI PLAN REPLAY / RULE EXECUTION\n\nExisting course / not constructed here\nNo fresh inference during replay\nP: pause / resume | Esc: close'
                if construction == 'gate':
                    left = 'ASSEMBLE + DRIVE / RULE EXECUTION\n\nTwo boxes / one continuous simulation\nNo teleport during execution\nP: pause / resume | Esc: close'
                if ramp:
                    left = 'RAMP CROSSING / RULE CONTROL\n\nStatic ramp / not built by the vehicle\nPhysical wheel contact / height 0.35 m\nP: pause / resume | Esc: close'
                if portable_ramp:
                    left = 'PORTABLE RAMP / RULE CONTROL\n\nLift > Carry > Place > Cross\nOne simulation / no teleport or weld\nShared top: 0.44 m / Slope: 16.3 deg\nP: pause / resume | Esc: close'
                if move_slope:
                    left = f'SINGLE {move_slope.upper()} SLOPE / RULE CONTROL\n\nLift > Carry > Place\nOffset pickup rail / no weld\nTop: 0.44 m / provisional mass: 1.5 kg\nP: pause / resume | Esc: close'
                if build_slopes:
                    left = 'BUILD TWO SLOPES / RULE CONTROL\n\nPlace UP > Place DOWN > Cross\nOne simulation / no teleport or weld\nTop: 0.44 m / nominal joint gap: 0.05 m\nP: pause / resume | Esc: close'
                right = ''
                ui['message'] = f"{pilot.stage} {'(PAUSED)' if ui['paused'] else ''} | {pilot.reason}"
            mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               viewport, left, right, context)
            mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               viewport, f'Blade: {lift_pos:.2f} m  |  Sim: {sim.data.time:.1f} s\n{ui["message"]}', '', context)
            if screenshot:
                from PIL import Image
                pixels = np.empty((height, width, 3), dtype=np.uint8)
                mujoco.mjr_readPixels(pixels, None, viewport, context)
                Image.fromarray(np.flipud(pixels)).save(screenshot)
                break
            glfw.swap_buffers(window)
            if smoke and now - started > 3:
                break
        return 0
    finally:
        recorder.stop('window_closed')
        if context:
            context.free()
        if window:
            glfw.destroy_window(window)
        glfw.terminate()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--screenshot', type=Path)
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--layout', type=Path, help='Preview a saved target layout; no driving or recording.')
    parser.add_argument('--build', action='store_true', help='Carry red box in an initially aligned straight lane.')
    parser.add_argument('--plan', type=Path, help='Replay a saved chat-authored plan on an existing course.')
    parser.add_argument('--assemble', action='store_true', help='Build both boxes then drive, without resetting the scene.')
    parser.add_argument('--ramp', action='store_true', help='Cross a static physical ramp; independent driving experiment.')
    parser.add_argument('--portable-ramp', action='store_true', help='Lift, transport, place and cross a free-body ramp.')
    parser.add_argument('--move-slope', choices=('up', 'down'), help='Carry one independent slope in an aligned lane.')
    parser.add_argument('--build-slopes', action='store_true', help='Place both slope parts and cross the assembled course.')
    args = parser.parse_args()
    if args.build_slopes and (args.move_slope or args.layout or args.plan or args.build or args.assemble or args.ramp or args.portable_ramp):
        parser.error('--build-slopes uses a fixed two-piece course')
    if args.move_slope and (args.layout or args.plan or args.build or args.assemble or args.ramp or args.portable_ramp):
        parser.error('--move-slope is an independent transport experiment')
    if args.ramp and (args.layout or args.plan or args.build or args.assemble):
        parser.error('--ramp is an independent driving experiment')
    if args.portable_ramp and (args.ramp or args.plan or args.build or args.assemble):
        parser.error('--portable-ramp accepts only an optional --layout')
    if args.assemble and not args.plan:
        parser.error('--assemble requires --plan')
    if args.plan and (args.layout or args.build):
        parser.error('--plan cannot be combined with --layout or --build')
    if args.build and not args.layout:
        parser.error('--build requires --layout')
    if args.layout:
        from layout import load_layout
        target_layout = load_layout(args.layout)
    else:
        target_layout = None
    driving_plan = None
    if args.plan:
        from ai_plan import load_plan
        driving_plan = load_plan(args.plan)
        target_layout = driving_plan['layout']
    raise SystemExit(run(args.screenshot, args.smoke, target_layout, 'gate' if args.assemble else args.build, driving_plan, args.ramp, args.portable_ramp, args.move_slope, args.build_slopes))
