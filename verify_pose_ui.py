"""Hidden-window scripted keyboard flow; not a human demonstration."""
import itertools
from unittest.mock import patch
import glfw
import app
import pose_task
from pose_dataset import inspect_episode


def main():
    state = {}
    original_create = glfw.create_window
    original_callback = glfw.set_key_callback
    original_attrib = glfw.get_window_attrib
    original_simulation = app.Simulation
    original_recorder = pose_task.PoseRecorder

    def create(*args):
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        state['window'] = original_create(*args)
        return state['window']

    def callback(window, fn):
        state['key'] = fn
        return original_callback(window, fn)

    def simulation(*args, **kwargs):
        state['sim'] = original_simulation(*args, **kwargs)
        return state['sim']

    def recorder(task, xml, folder):
        state['recorder'] = original_recorder(task, xml, app.ROOT/'.test-results/pose-ui', source='scripted_test')
        return state['recorder']

    def key(code, action):
        state['key'](state['window'], code, 0, action, 0)

    def poll():
        sim, rec = state['sim'], state['recorder']
        if not state.get('started'):
            key(glfw.KEY_F, glfw.PRESS)
            key(glfw.KEY_Q, glfw.PRESS)
            key(glfw.KEY_S, glfw.PRESS)
            state['started'] = True
        if sim.data.body('dozer').xpos[0] < -.97:
            key(glfw.KEY_S, glfw.RELEASE)
        if rec.task.done or sim.data.time > 15:
            glfw.set_window_should_close(state['window'], True)

    ticks = itertools.count(0, .025)
    with patch.object(glfw, 'create_window', create), patch.object(glfw, 'set_key_callback', callback), \
         patch.object(glfw, 'get_window_attrib', lambda w, a: 1 if a == glfw.FOCUSED else original_attrib(w, a)), \
         patch.object(glfw, 'poll_events', poll), patch.object(app, 'Simulation', simulation), \
         patch.object(pose_task, 'PoseRecorder', recorder), patch.object(app.time, 'perf_counter', lambda: next(ticks)):
        app.run(pose_case='back')
    rec = state['recorder']
    assert rec.task.success and rec.saved_success and rec.file is None
    report = inspect_episode(rec.path)
    assert report['success'] and not report['eligible_for_training']
    print('PASS: F starts; S + Q reverse; one-second hold; automatic save; complete goal-aware episode.')


if __name__ == '__main__': main()
