"""
Environment for DJI Robomaster AI Challenge.
"""

import math
import gym
from gym import spaces, logger
from utils import *
#from gym.envs.classic_control import rendering
import rendering
from gym.utils import seeding
from gym.envs.DJI.Objects import *
import numpy as np
import cv2

"""
Currently doesn't fit into the OpenAI gym model

For future modification, consider definition of action set
"""
class RobomasterEnv(gym.Env):
	# """
	# Description:
	# 	A robot placed in a battlefield to face off against an enemy robot. Robots fire at each other to deal damage.
	# 	Score is calculated at the end of a game to determine the winner.
	#
	# Source:
	# 	This environment is a simplified version of the DJI Robomaster AI Challenge field.
	#
	# Observation:
	# 	Type: Box(4)
	# 	Num 	Observation 			Min 		Max
	# 	0		Robot X Position		0.0			800.0
	# 	1		Robot Y Position		0.0			500.0
	# 	2		Enemy X Position		0.0			800.0
	# 	3		Enemy Y Position		0.0			500.0
	#
	# Actions:
	# 	Type: Box(2)
	# 	Num 	Action 			Min			Max
	# 	0		X Position		0			800
	# 	1		Y Position		0			500
	#
	# Reward:
	# 	Reward is 1 for every shot taken at the enemy.
	# 	Reward is -1 for every shot taken from the enemy.
	#
	# Starting State:
	# 	Initial Robot X and Robot Y are both 0.0.
	# 	Initial Enemy X and Enemy Y are 800.0 and 500.0 respectively.
	#
	# Episode Termination:
	# 	Robot health is 0.
	# 	Enemy health is 0.
	# 	Episode length is greater than 300.
	# """
	#
	# metadata = {
	# 	'render.modes': ['human', 'rgb_array'],
	# 	'video.frames_per_second': 50
	# }

	# Defining course dimensions
	start_zone_sidelength = 133
	width = 800.0
	height = 500.0
	tau = .01

	def __init__(self):

		# Record time
		self.game_time = 0

		self.characters = {
		    "robots": [],
			"obstacles": [],
			"zones": [],
			"bullets": [],
		}

		# Initialize teams
		BLUE = Team((0, 0, 1), "BLUE")
		RED = Team((1, 0, 0), "RED")
		BLUE.enemy, RED.enemy = RED, BLUE
		self.my_team, self.enemy_team = BLUE, RED

		# Initialize robots
		myRobot = AttackRobot(self, BLUE, Point(350, 320), 0)
		enemyRobot = DummyRobot(self, RED, Point(750, 450), 180)
		myRobot.load(40)
		enemyRobot.load(40)
		self.characters['robots'] = [myRobot, enemyRobot]

		# Defining course obstacles
		obstacleParams = [(Point(0, 220), 80, 30), (Point(120, 370), 80, 30),
			(Point(180, 150), 30, 120), (Point(310, 0), 30, 200)]
		self.characters['obstacles'] = [Obstacle(p[0], p[1], p[2])
		    for p in obstacleParams]

		# Team start areas
		myStartArea = StartingZone(self, BLUE)
		enemyStartArea = StartingZone(self, RED)
		self.characters['zones'] = [myStartArea, enemyStartArea]

		self.background = Rectangle(Point(0, 0), self.width, self.height, 0)

		self.viewer = None
		# Load area

		# Defense Buff area

		# # Bonus area
		# self.bonus_bounds = [self.width / 2 - 30, self.width / 2 + 30, self.height / 2 + 30, self.height / 2 - 30]
		#
		# # Defining action space
		# act_low = np.array([0, 0])
		# act_high = np.array([800, 500])
		# self.action_space = spaces.Box(act_low, act_high, dtype=np.float32)
		#
		# # Defining observation space
		# minx, miny = 0.0, 0.0
		# maxx, maxy = 800.0, 500.0
		# obs_low = np.array([minx, miny, minx, miny])
		# obs_high = np.array([maxx, maxy, maxx, maxy])
		# self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)
		#
		# self.seed()
		# self.viewer = None
		# self.state = None

	# def seed(self, seed=None):
	# 	self.np_random, seed = seeding.np_random(seed)
	# 	return [seed]

	def state(self):
		return []

	def actables(self):
		return self.characters['robots'] + self.characters['bullets']

	def unpenetrables(self):
		return self.characters['robots'] + self.characters['obstacles']

	def impermissibles(self, robot):
		return list(filter(lambda r: not r == robot, self.characters['robots'])) \
		    + self.characters['obstacles'] \
		    + list(filter(lambda z: not z.permissble(robot.team), self.characters['zones']))

	# Moves the game 1 timestep defined by self.tau
	def step(self):
		# """
		# robot_action: a point(x, y) the robot will travel to
		# enemy_action: a point(x, y) the enemy robot will travel to
		# """
		self.game_time += RobomasterEnv.tau
		for char in self.actables():
			char.act()

		# assert self.action_space.contains(robot_action),  "%r (%s) invalid"%(robot_action, type(robot_action))
		#
		# state = self.state
		# self.game_time += self.tau
		#
		# # Updates positions of robots
		# self.robot.move(self.enemy, self, robot_action[0], robot_action[1], self.obstacles, self.tau)
		#
		# #enemy_action =  self.enemy.random_action() # Temporary enemy action for testing
		# #enemy_action = [700, 0]
		# self.enemy.move(self.robot, self, enemy_action[0], enemy_action[1], self.obstacles, self.tau)
		#
		# # Update gun angle of robot
		# self.robot.aim(self.enemy, self.obstacles)
		#
		# # Update state
		# self.state = [self.robot.x, self.robot.y, self.robot.gun_angle, self.enemy.x, self.enemy.y, self.enemy.gun_angle]

		# NOT YET IMPLEMENTED
		reward = 0.0

		# NOT YET IMPLEMENTED
		done = False

		return np.array(self.state), reward, done, {}

	# Resets the field to the starting positions
	def reset(self):
		self.__init__()
		# self.robot.x, self.robot.y = self.robot.width / 2, self.robot.length / 2
		# self.enemy.x, self.enemy.y = self.width - self.enemy.width / 2, self.height - self.enemy.length / 2
		# self.robot.gun_angle, self.enemy.gun_angle = 0.0, 0.0
		#
		# self.game_time = 0
		#
		# self.state = [self.robot.x, self.robot.y, self.robot.gun_angle, self.enemy.x, self.enemy.y, self.enemy.gun_angle]
		return np.array(self.state)

	# Renders the field for human observation
	def render(self, mode='human'):
		screen_width = 800
		screen_height = 500

		world_width = self.width
		scale = screen_width / world_width

		# # Function to make box obstacles
		# def make_obstacle(obstacle):
		# 	l, r, t, b = obstacle.l, obstacle.r, obstacle.t, obstacle.b
		# 	obstacle = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		#
		# 	return obstacle
		#
		# def make_zone(zone, color):
		# 	l, r, t, b = zone
		# 	zone = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		# 	zone.set_color(color[0], color[1], color[2])
		#
		# 	return zone

		# Creates viewer object, robots, and obstacles
		if self.viewer is None:
			self.viewer = rendering.Viewer(screen_width, screen_height)

		self.viewer.resetGeoms()
		# 	# Add starting zones
		# 	self.viewer.add_geom(make_zone(self.team1_start, [0,0,0.5]))
		# 	self.viewer.add_geom(make_zone(self.team2_start, [0.5,0,0]))
		#
		# 	# Add bonus zone
		# 	self.viewer.add_geom(make_zone(self.bonus_bounds, [0.8, 0.8, 0]))
		#
		# 	# Add robot geometry
		# 	l, r, t, b = -self.robot.width / 2, self.robot.width / 2, -self.robot.length / 2, self.robot.length / 2
		# 	robot = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		# 	robot.set_color(0, 0, 1)
		# 	self.robot_trans = rendering.Transform()
		# 	robot.add_attr(self.robot_trans)
		#
		# 	l, r, t, b = -self.robot.gun_width / 2, self.robot.gun_width / 2, -self.robot.gun_width / 2, self.robot.gun_length - self.robot.gun_width / 2
		# 	robot_gun = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		# 	self.robot_guntrans = rendering.Transform()
		# 	robot_gun.add_attr(self.robot_guntrans)
		#
		# 	self.viewer.add_geom(robot)
		# 	self.viewer.add_geom(robot_gun)
		#
		# 	# Add enemy geometry
		# 	l, r, t, b = -self.enemy.width / 2, self.enemy.width / 2, -self.enemy.length / 2, self.enemy.length / 2
		# 	enemy = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		# 	enemy.set_color(1, 0, 0)
		# 	self.enemy_trans = rendering.Transform()
		# 	enemy.add_attr(self.enemy_trans)
		#
		# 	l, r, t, b = -self.enemy.gun_width / 2, self.enemy.gun_width / 2, -self.enemy.gun_width / 2, self.enemy.gun_length - self.enemy.gun_width / 2
		# 	enemy_gun = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
		# 	self.enemy_guntrans = rendering.Transform()
		# 	enemy_gun.add_attr(self.enemy_guntrans)
		#
		# 	self.viewer.add_geom(enemy)
		# 	self.viewer.add_geom(enemy_gun)
		#
		# 	# Add obstacles
		# 	for obstacle in self.obstacles:
		# 		self.viewer.add_geom(make_obstacle(obstacle))
		#
		# if self.state is None: return None
		#
		# x = self.state
		#
		# # Render location for robot
		# robotx, roboty = x[0]*scale, x[1]*scale
		# gun_angle = x[2]*np.pi/180
		# self.robot_trans.set_translation(robotx, roboty)
		# self.robot_trans.set_rotation((self.robot.angle - 90)*np.pi/180)
		# self.robot_guntrans.set_translation(robotx, roboty)
		# self.robot_guntrans.set_rotation(gun_angle)
		#
		# # Render location for enemy
		# enemyx, enemyy = x[3]*scale, x[4]*scale
		# enemy_gun_angle = x[5]*np.pi/180
		# self.enemy_trans.set_translation(enemyx, enemyy)
		# self.enemy_guntrans.set_translation(enemyx, enemyy)
		# self.enemy_guntrans.set_rotation(enemy_gun_angle)
		#

		self.viewer.add_geom(self.background.render((1, 1, 1)))

		for zone in self.characters['zones']:
			geom = zone.render()
			if geom:
				self.viewer.add_geom(geom)

		for obs in self.characters['obstacles']:
			geom = obs.render()
			if geom:
				self.viewer.add_geom(geom)

		for robot in self.characters['robots']:
			geoms = robot.render()
			if geoms:
				for geom in geoms:
					self.viewer.add_geom(geom)

		for bullet in self.characters['bullets']:
			geom = bullet.render()
			if geom:
				self.viewer.add_geom(geom)

		return self.viewer.render(return_rgb_array = mode == 'rgb_array')

	def isObstructed(self, rec, robot):
		for ob in self.impermissibles(robot):
			if ob.intersects(rec) or ob.intersects(robot.gun):
				return True
		for v in robot.vertices:
			if not self.isLegal(v):
				return True
		return False

	def isLegal(self, point):
		return point.x >= 0 and point.x <= self.width and point.y >= 0 and point.y <= self.height

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None
