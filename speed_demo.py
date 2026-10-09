"""Flat straight speed trial; existing vehicle and motor limits, no learning."""
import argparse
import json
import time
import xml.etree.ElementTree as ET
import numpy as np
import glfw
import mujoco
from app import ROOT, Simulation


def make_sim():
    root = ET.parse(ROOT/'scene_fork_tracks.xml').getroot()
    world = root.find('worldbody')
    for body in list(world.findall('body')):
        if body.get('name') != 'dozer': world.remove(body)
    for geom in list(world.findall('geom')):
        if geom.get('name') != 'floor': world.remove(geom)
    world.find("geom[@name='floor']").set('size', '40 6 .1')
    world.find("body[@name='dozer']").set('pos', '0 0 .3')
    for x in range(-5, 36):
        ET.SubElement(world, 'geom', type='box', pos=f'{x} 0 .002', size='.015 2 .002',
                      rgba='.2 .5 .6 1', contype='0', conaffinity='0')
    return Simulation(scene_xml=ET.tostring(root, encoding='unicode'))


def tick(sim, motor_limit):
    sim.step((1.6 if motor_limit else 1.0) if sim.data.time < 10 else 0, 0, 0)
    speed = float(np.linalg.norm(sim.data.qvel[:2]))
    assert np.isfinite(sim.data.qpos).all(), 'Non-finite state'
    return speed


def measure(motor_limit):
    sim = make_sim(); speeds = []; cruise = []
    while sim.data.time < 13:
        speed = tick(sim, motor_limit); speeds.append(speed)
        if 7 < sim.data.time < 10: cruise.append(speed)
    return {'mode': 'existing_motor_limit' if motor_limit else 'normal_W_key',
            'wheel_command_rad_s': 8 if motor_limit else 5,
            'peak_m_s': max(speeds), 'cruise_m_s': float(np.mean(cruise)),
            'peak_km_h': max(speeds)*3.6, 'stopped_speed_m_s': speeds[-1],
            'note': 'Empty vehicle on flat ground; four-wheel contact proxy. Motor-limit mode bypasses normal keyboard scaling but preserves XML motor limits. Not a real machine specification.'}


def view(motor_limit):
    if not glfw.init(): raise RuntimeError('GLFW unavailable')
    window = None; context = None
    try:
        window = glfw.create_window(1280, 800, 'Speed trial | '+('MOTOR LIMIT 8 rad/s' if motor_limit else 'NORMAL W 5 rad/s'), None, None)
        if not window: raise RuntimeError('Window unavailable')
        glfw.make_context_current(window); glfw.swap_interval(1)
        sim = make_sim(); camera = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(camera)
        camera.distance = 5; camera.azimuth = 135; camera.elevation = -25
        scene = mujoco.MjvScene(sim.model, maxgeom=3000); option = mujoco.MjvOption()
        context = mujoco.MjrContext(sim.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        last = time.perf_counter(); accumulator = 0; peak = speed = 0
        running = False
        def key_callback(window, key, scancode, action, mods):
            nonlocal running, peak, speed, accumulator
            if action != glfw.PRESS: return
            if key == glfw.KEY_SPACE:
                running = not running
                accumulator = 0
            elif key == glfw.KEY_R:
                sim.reset(); peak = speed = accumulator = 0; running = False
        glfw.set_key_callback(window, key_callback)
        while not glfw.window_should_close(window):
            glfw.poll_events()
            if glfw.get_key(window, glfw.KEY_ESCAPE) == glfw.PRESS: break
            now = time.perf_counter(); accumulator += min(now-last, .1); last = now
            while accumulator >= .02:
                if running and sim.data.time < 13:
                    speed = tick(sim, motor_limit); peak = max(peak, speed)
                accumulator -= .02
            sim.update_tracks(); camera.lookat[:] = sim.data.body('dozer').xpos
            width, height = glfw.get_framebuffer_size(window)
            if not width or not height: time.sleep(.02); continue
            viewport = mujoco.MjrRect(0, 0, width, height)
            mujoco.mjv_updateScene(sim.model, sim.data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            mode = 'MOTOR LIMIT / 8 rad/s (above normal keyboard input)' if motor_limit else 'NORMAL W KEY / 5 rad/s'
            phase = 'FULL THROTTLE' if sim.data.time < 10 else 'BRAKING' if sim.data.time < 13 else 'FINISHED'
            if not running and sim.data.time < 13: phase = 'PRESS SPACE TO START / RESUME'
            text = f'{mode}\n{phase} | {sim.data.time:.1f} s\nSpeed: {speed*3.6:.2f} km/h ({speed:.3f} m/s)\nPeak: {peak*3.6:.2f} km/h\nSpace: start/pause | R: reset | Esc: close\nFloor marks: 1 m\nPhysics simulation / no AI / no training'
            mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, text, '', context)
            glfw.swap_buffers(window)
    finally:
        if context: context.free()
        if window: glfw.destroy_window(window)
        glfw.terminate()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motor-limit', action='store_true')
    parser.add_argument('--measure', action='store_true')
    args = parser.parse_args()
    if args.measure: print(json.dumps([measure(False), measure(True)], indent=2))
    else: view(args.motor_limit)
