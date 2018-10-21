import gym
import numpy as np
import time

env = gym.make('Robomaster-v0')
env.reset()
env.obstacle_width = 50
env.obstacle_height = 50
for _ in range(1000):
	env.render()
	env.step(np.array([45, 50]))
	time.sleep(.5)

