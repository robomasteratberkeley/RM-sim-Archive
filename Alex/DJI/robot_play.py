import gym
import numpy as np
import time
import cv2
from waypointing import find_path
from waypointing import new_path

env = gym.make('Robomaster-v0')
env.reset()
env.obstacle_width = 50
env.obstacle_height = 50

path = find_path(env.robot, env, [700,400])
destination = np.array([0,0])
for _ in range(1000):
	env.render()
	if path is not None and len(path) > 0:
		destination = path[0]
	# print(destination, env.robot.x, env.robot.y)
	env.step(destination)
	path = new_path(env.robot, path)
	# time.sleep(.1)
