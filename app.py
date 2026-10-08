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
OBJECTS = ['dozer', 'blade', 'block_1', 'block_2', 'block_3', 'block_4', 'ball']
CONTROL_DT = 0.02


class Simulation:
    def __init__(self, attachment='fork', layout=None, construction=False):
        self.attachment = attachment
        self.scene_path = ROOT / ('scene_fork_tracks.xml' if attachment == 'fork' else 'scene_tracks.xml')
        if layout is None:
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


def run(screenshot=None, smoke=False, layout=None, construction=False):
    if not glfw.init():
        raise RuntimeError('OpenGL window could not initialize.')
    window = None
    context = None
    recorder = Recorder()
    try:
        if screenshot:
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        glfw.window_hint(glfw.SAMPLES, 4)
        title = 'Auto Transport | Faster cruise / RED BOX ONLY' if construction else 'Layout Preview | NOT a construction result' if layout else 'Bulldozer Lab | WASD + Space / Shift'
        window = glfw.create_window(1280, 800, title, None, None)
        if not window:
            raise RuntimeError('Could not create OpenGL window. Check the graphics driver.')
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        sim = Simulation(layout=layout, construction=construction)
        pilot = None
        if construction:
            from transport import Transport
            pilot = Transport(sim, layout['blocks'][0])
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
        ui = {'paused': bool(layout) and not construction, 'follow': False, 'message': 'Lower forks, slide under a box, then Space to lift. Q = slow drive.',
              'mouse': None}

        def on_key(win, key, scancode, action, mods):
            nonlocal sim, scene, context
            if action == glfw.PRESS:
                if layout and key not in ((glfw.KEY_ESCAPE, glfw.KEY_1, glfw.KEY_2, glfw.KEY_P) if construction else (glfw.KEY_ESCAPE, glfw.KEY_1, glfw.KEY_2)):
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
            if layout and not construction:
                left = 'TARGET LAYOUT PREVIEW\n\nDesired final positions only\nNo autonomous construction / No AI\n\nCamera: drag / wheel\nView: 1 overview / 2 top\nClose: Esc'
                right = ''
                ui['message'] = 'Design preview. Boxes were placed directly for visualization, not moved by the robot.'
            if pilot:
                left = 'AUTO TRANSPORT / RULE CONTROL\n\nRED BOX ONLY / STRAIGHT LANE\nP: pause / resume | Esc: close\nView: 1 / 2 | Mouse: camera'
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
    args = parser.parse_args()
    if args.build and not args.layout:
        parser.error('--build requires --layout')
    if args.layout:
        from layout import load_layout
        target_layout = load_layout(args.layout)
    else:
        target_layout = None
    raise SystemExit(run(args.screenshot, args.smoke, target_layout, args.build))
