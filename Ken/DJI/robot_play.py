import gym
import numpy as np
import time
import cv2

import waypointing as wp

env = gym.make('Robomaster-v0').unwrapped

#path = wp.find_path(env.robot, env, [400,300])
#path_for_enemy = wp.find_path(env.enemy, env, [600,400])
# path = wp.patrol(env.robot, env, 4, [[40, 400], [100, 300], [400, 300], [400, 450], [40, 400]])
# robot_destination = np.array([0,0])
#enemy_destination = np.array([0,0])
for _ in range(18000):
	env.render()
	env.step()
	# if path is not None and len(path) > 0:
	# 	robot_destination = path[0]
	# env.step(robot_destination, np.array([600, 400]))
	# path = wp.new_path(env.robot, path)
	#path_for_enemy = new_path(env.enemy, path_for_enemy)
	# time.sleep(.1)
	if env.finished:
		break
