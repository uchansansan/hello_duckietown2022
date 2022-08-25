#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""
from PIL import Image
import argparse
import sys

import gym
import numpy as np
import pyglet
from DuckiebotControl import DuckiebotControl
from pyglet.window import key

from gym_duckietown.envs import DuckietownEnv

# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument("--env-name", default=None)
parser.add_argument("--map-name", default="udem1")
parser.add_argument("--distortion", default=False, action="store_true")
parser.add_argument("--camera_rand", default=False, action="store_true")
parser.add_argument("--draw-curve", action="store_true", help="draw the lane following curve")
parser.add_argument("--draw-bbox", action="store_true", help="draw collision detection bounding boxes")
parser.add_argument("--domain-rand", action="store_true", help="enable domain randomization")
parser.add_argument("--dynamics_rand", action="store_true", help="enable dynamics randomization")
parser.add_argument("--frame-skip", default=1, type=int, help="number of frames to skip")
parser.add_argument("--seed", default=1, type=int, help="seed")
parser.add_argument("--file", type=str)
parser.add_argument("--line", type=int)
args = parser.parse_args()

if args.env_name and args.env_name.find("Duckietown") != -1:
    env = DuckietownEnv(
        seed=args.seed,
        map_name=args.map_name,
        draw_curve=args.draw_curve,
        draw_bbox=args.draw_bbox,
        domain_rand=args.domain_rand,
        frame_skip=args.frame_skip,
        distortion=args.distortion,
        camera_rand=args.camera_rand,
        dynamics_rand=args.dynamics_rand,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()

duckiebot = DuckiebotControl(args.frame_skip)



# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

commands = []

if args.file:
    inputCommands = open(args.file, "r")
    for line in inputCommands:
        if len(line):
            direction, tics = line.split()[0], line.split()[1]
            commands += list(direction) * int(tics)

#print(commands)


def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    action = np.array([0.0, 0.0])
    if args.file:
        action = np.array([0.0, 0.0])
        global commands
        if len(commands):
            if commands[0] == 'F':
                action += np.array([0.44, 0.0])
            if commands[0] == 'B':
                action -= np.array([0.44, 0])
            if commands[0] == 'R':
                action -= np.array([0, 1])
            if commands[0] == 'L':
                action += np.array([0, 1])
            commands = commands[1::]

    elif args.line == 1:
        action = np.array([0.0, 0.0])
        lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
        distance_to_road_center = lane_pose.dist
        angle_from_straight_in_rads = lane_pose.angle_rad

        # print(f"{distance_to_road_center=}, {angle_from_straight_in_rads=}")

        global duckiebot
        duckiebot.update(angle_from_straight_in_rads, distance_to_road_center)
        action = duckiebot.get_action()
        obs, reward, done, info = env.step(action)
        if done:
            env.reset()
            env.render()
    else:

        action = np.array([0.0, 0.0])

        if key_handler[key.UP]:
            action += np.array([0.44, 0.0])
        if key_handler[key.DOWN]:
            action -= np.array([0.44, 0])
        if key_handler[key.LEFT]:
            action += np.array([0, 1])
        if key_handler[key.RIGHT]:
            action -= np.array([0, 1])
        if key_handler[key.SPACE]:
            action = np.array([0, 0])


        # Speed boost
        if key_handler[key.LSHIFT]:
            action *= 1.5

    obs, reward, done, info = env.step(action)

    print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))



    if done:
        env.reset()
        env.render()
    env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()