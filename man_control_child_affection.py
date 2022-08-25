import pyglet
import numpy as np

from pyglet.window import key
from gym_duckietown.envs import DuckietownEnv

env = DuckietownEnv(map_name='udem1')
key_handler = key.KeyStateHandler()
env.reset()
env.render()

env.unwrapped.window.push_handlers(key_handler)


def update(dt):
    action = np.array([0, 0])

    if key_handler[key.UP]:
        action += np.array([1, 0])
    if key_handler[key.DOWN]:
        action -= np.array([1, 0])
    if key_handler[key.LEFT]:
        action += np.array([0, 1])
    if key_handler[key.RIGHT]:
        action -= np.array([0, 1])
    if key_handler[key.LSHIFT]:
        action *= 2

    obs, reward, done, info = env.step(action)
    
    if done:
        env.reset()
        env.render()
    env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)
pyglet.app.run()
env.close()
