"""
Environment for DJI Robomaster AI Challenge.
"""

import math
import gym
from gym import spaces, logger
from gym.envs.classic_control import rendering
from gym.utils import seeding
import numpy as np

class RobomasterEnv(gym.Env):
	"""
	Description:
		A robot placed in a battlefield to face off against an enemy robot. Robots fire at each other to deal damage.
		Score is calculated at the end of a game to determine the winner.

	Source:
		This environment is a simplified version of the DJI Robomaster AI Challenge field.
	
	Observation:
		Type: Box(4)
		Num 	Observation 			Min 		Max
		0		Robot X Position		0.0			800.0
		1		Robot Y Position		0.0			500.0
		2		Enemy X Position		0.0			800.0
		3		Enemy Y Position		0.0			500.0

	Actions:
		Type: Box(2)
		Num 	Action 					Min		Max
		0		Movement Direction		0.0°	360.0°
		1		Movement Speed			0.0		300.0

	Reward:
		Reward is 1 for every shot taken at the enemy.
		Reward is -1 for every shot taken from the enemy.

	Starting State:
		Initial Robot X and Robot Y are both 0.0. 
		Initial Enemy X and Enemy Y are 800.0 and 500.0 respectively.

	Episode Termination:
		Robot health is 0.
		Enemy health is 0.
		Episode length is greater than 300.
	"""

	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 50
	}

	def __init__(self):
		self.robot_health = 100
		self.enemy_health = 100
		self.tau = .1 # seconds between state updates
		self.game_time = 0

		# Defining course dimensions
		self.width = 800.0
		self.height = 500.0

		# Defining robot dimensions
		self.robot_width = 30.0
		self.robot_length = 50.0
		self.gun_width = self.robot_width / 2
		self.gun_length = self.robot_length

		# Defining course obstacles
		self.obstacle_width = 150.0
		self.obstacle_height = 150.0

		self.obstacles = [[200.0 + self.obstacle_width / 2, self.obstacle_height / 2], \
							[500.0 + self.obstacle_width / 2, self.obstacle_height / 2],
							[200.0 + self.obstacle_width / 2, self.height - self.obstacle_height / 2],
							[500.0 + self.obstacle_width / 2, self.height - self.obstacle_height / 2]]


		# Defining action space
		act_low = np.array([0.0, 0.0])
		act_high = np.array([360.0, 300.0])
		self.action_space = spaces.Box(act_low, act_high, dtype=np.float32)


		# Defining observation space
		minx, miny = 0.0, 0.9
		maxx, maxy = 800.0, 500.0
		obs_low = np.array([minx, miny, minx, miny])
		obs_high = np.array([maxx, maxy, maxx, maxy])
		self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

		self.seed()
		self.viewer = None
		self.state = None

	def seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	# Moves the game 1 timestep defined by self.tau
	def step(self, action):
		assert self.action_space.contains(action),  "%r (%s) invalid"%(action, type(action))

		state = self.state
		self.game_time += self.tau

		# Updates positions of robots
		def move(robot_position, angle, speed):
			angle = angle * np.pi / 180
			timestep = self.tau
			x = robot_position[0]
			y = robot_position[1]

			x = x + speed*timestep * np.cos(angle)
			y = y + speed*timestep * np.sin(angle)

			# Check for collisions
			if x < self.robot_width / 2:
				x = self.robot_width / 2
			elif x + self.robot_width / 2 > self.width:
				x = self.width - self.robot_width / 2

			if y < self.robot_length / 2:
				y = self.robot_length / 2
			elif y + self.robot_length / 2 > self.height:
				y = self.height - self.robot_length / 2

			# NOT FULLY IMPLEMENTED.
			# NEED TO CHECK: EACH OTHER

			# Check for obstacles
			for obstacle in self.obstacles:
				l = obstacle[0] - self.obstacle_width / 2 - self.robot_width / 2
				r = obstacle[0] + self.obstacle_width / 2 + self.robot_width / 2
				b = obstacle[1] - self.obstacle_height / 2 - self.robot_length / 2
				t = obstacle[1] + self.obstacle_height / 2 + self.robot_length / 2
				
				# Very basic collision management. Not physically accurate.
				if l < x < r and b < y < t:
					if robot_position[0] <= l:
						x = l
					elif robot_position[0] >= r:
						x = r
					if robot_position[1] <= b:
						y = b
					elif robot_position[1] >= t:
						y = t


			return x, y

		def random_action():
			return np.random.random_sample(2,)*[360.0, 150.0]

		robot_x, robot_y = move([state[0], state[1]], action[0], action[1])

		enemy_action =  random_action() # Temporary enemy action for testing
		enemy_x, enemy_y = move([state[3], state[4]], enemy_action[0], enemy_action[1])
		def find_angle(x1, y1, x2, y2):
			if x1 == x2:
				if y1 > y2:
					return 180
				else:
					return 0
			return (np.arctan((y1 - y2) / (x1 - x2)) * 180 / np.pi - 90) % 360
		robot_gunangle = find_angle(robot_x, robot_y, enemy_x, enemy_y)

		# Check for obstacles blocking line of sight
		for obstacle in self.obstacles:
			l = obstacle[0] - self.obstacle_width / 2
			r = obstacle[0] + self.obstacle_width / 2
			b = obstacle[1] - self.obstacle_height / 2
			t = obstacle[1] + self.obstacle_height / 2
			corners = [[l, b], [l, t], [r, b], [r, t]]
			def check_angle(x1, y1, x2, y2, x3, y3, robot_gunangle):
				upper_angle = find_angle(robot_x, robot_y, x2, y2)
				lower_angle = find_angle(robot_x, robot_y, x3, y3)
				if lower_angle < robot_gunangle < upper_angle:
					return state[2]
				else:
					return robot_gunangle
			if robot_x < l and enemy_x > l:
				if robot_y > t:
					robot_gunangle = check_angle(robot_x, robot_y, r, t, l, b, robot_gunangle)
				elif robot_y < b:
					robot_gunangle = check_angle(robot_x, robot_y, l, t, r, b, robot_gunangle)
				else:
					robot_gunangle = check_angle(robot_x, robot_y, l, t, l, b, robot_gunangle)
			elif robot_x > r and enemy_x < r:
				if robot_y > t:
					robot_gunangle = check_angle(robot_x, robot_y, r, b, l, t, robot_gunangle)
				elif robot_y < b:
					robot_gunangle = check_angle(robot_x, robot_y, l, b, r, t, robot_gunangle)
				else:
					robot_gunangle = check_angle(robot_x, robot_y, r, b, r, t, robot_gunangle)
			else:
				if robot_y > t:
					robot_gunangle = check_angle(robot_x, robot_y, r, t, l, t, robot_gunangle)
				else:
					upper_angle = find_angle(robot_x, robot_y, r, b)
					lower_angle = find_angle(robot_x, robot_y, l, b)
					if robot_gunangle > upper_angle or robot_gunangle < lower_angle:
						robot_gunangle = state[2]

		self.state = [robot_x, robot_y, robot_gunangle, enemy_x, enemy_y, state[5]]
		
		# NOT YET IMPLEMENTED
		reward = 0.0

		# NOT YET IMPLEMENTED
		done = False

		return np.array(self.state), reward, done, {}

	# Resets the field to the starting positions
	def reset(self):
		robotx, roboty = self.robot_width / 2, self.robot_length / 2
		enemyx, enemyy = self.width - self.robot_width / 2, self.height - self.robot_length / 2
		robot_gun_angle, enemy_gun_angle = 0.0, 0.0
		self.state = [robotx, roboty, robot_gun_angle, enemyx, enemyy, enemy_gun_angle]
		return np.array(self.state)

	# Renders the field for human observation
	def render(self, mode='human'):
		screen_width = 800.0
		screen_height = 500.0

		world_width = self.width
		scale = screen_width / world_width

		# Function to make box obstacles
		def make_obstacle(obstacle_position):
			l, r, t, b = -self.obstacle_width / 2, self.obstacle_width / 2, -self.obstacle_height / 2, self.obstacle_height / 2
			obstacle = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
			obstacletrans = rendering.Transform()
			obstacletrans.set_translation(obstacle_position[0], obstacle_position[1])
			obstacle.add_attr(obstacletrans)

			return obstacle

		# Creates viewer object, robots, and obstacles
		if self.viewer is None:
			self.viewer = rendering.Viewer(screen_width, screen_height)

			# Add robot geometry
			l, r, t, b = -self.robot_width / 2, self.robot_width / 2, -self.robot_length / 2, self.robot_length / 2
			robot = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
			robot.set_color(0, 0, 1)
			self.robot_trans = rendering.Transform()
			robot.add_attr(self.robot_trans)

			l, r, t, b = -self.gun_width / 2, self.gun_width / 2, -self.gun_width / 2, self.gun_length - self.gun_width / 2
			robot_gun = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
			self.robot_guntrans = rendering.Transform()
			robot_gun.add_attr(self.robot_guntrans)
			robot_gun.add_attr(self.robot_trans)

			self.viewer.add_geom(robot)
			self.viewer.add_geom(robot_gun)

			# Add enemy geometry
			l, r, t, b = -self.robot_width / 2, self.robot_width / 2, -self.robot_length / 2, self.robot_length / 2
			enemy = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
			enemy.set_color(1, 0, 0)
			self.enemy_trans = rendering.Transform()
			enemy.add_attr(self.enemy_trans)

			l, r, t, b = -self.gun_width / 2, self.gun_width / 2, -self.gun_width / 2, self.gun_length - self.gun_width / 2
			enemy_gun = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
			self.enemy_guntrans = rendering.Transform()
			enemy_gun.add_attr(self.enemy_guntrans)
			enemy_gun.add_attr(self.enemy_trans)

			self.viewer.add_geom(enemy)
			self.viewer.add_geom(enemy_gun)

			# Add obstacles
			for obstacle_position in self.obstacles:
				obstacle = make_obstacle(obstacle_position)
				self.viewer.add_geom(obstacle)

		if self.state is None: return None

		x = self.state

		# Render location for robot
		robotx, roboty = x[0]*scale, x[1]*scale
		gun_angle = x[2]*np.pi/180
		self.robot_trans.set_translation(robotx, roboty)
		self.robot_guntrans.set_rotation(gun_angle)

		# Debug printing
		# if self.game_time % 10 <= self.tau:
		# 	print(self.state[2])

		# Render location for enemy
		enemyx, enemyy = x[3]*scale, x[4]*scale
		enemy_gun_angle = x[5]*np.pi/180
		self.enemy_trans.set_translation(enemyx, enemyy)
		self.enemy_guntrans.set_rotation(enemy_gun_angle)

		return self.viewer.render(return_rgb_array = mode == 'rgb_array')

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None


