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
	width = 800
	height = 500
	tau = 1
	full_time = 30000

	def __init__(self):

		# Record time
		self.game_time = 0
		self.finished = False

		self.characters = {
			"obstacles": [],
			"zones": [],
			"robots": [],
			"bullets": [],
		}

		# Initialize teams
		BLUE = Team("BLUE")
		RED = Team("RED")
		BLUE.enemy, RED.enemy = RED, BLUE
		self.my_team, self.enemy_team = BLUE, RED

		# Initialize robots
		myRobot = AttackRobot(self, BLUE, Point(170, 295), 0)
		enemyRobot = ManualControlRobot("OSPWADBR", self, RED, Point(250, 110), 0)
		myRobot.load(5)
		enemyRobot.load(40)
		self.characters['robots'] = [myRobot, enemyRobot]
		for i in range(len(self.characters['robots'])):
			self.characters['robots'][i].id = i

		# Defining course obstacles
		self.characters['obstacles'] = [Obstacle(p[0], p[1], p[2])
		    for p in [(Point(325, 0), 25, 100), (Point(450, 400), 25, 100),
				(Point(350, 238), 100, 25), (Point(580, 100), 100, 25),
				(Point(120, 375), 100, 25), (Point(140, 140), 25, 100),
				(Point(635, 260), 25, 100)]]

		# Team start areas
		self.startingZones = [StartingZone(p[0], p[1]) for p in [(Point(0, 0), BLUE),
		(Point(700, 0), BLUE), (Point(0, 400), RED), (Point(700, 400), RED)]]

		self.defenseBuffZones = [DefenseBuffZone(p[0], p[1]) for p in [
		(Point(120, 275), BLUE), (Point(580, 125), RED)]]

		self.loadingZones = [LoadingZone(p[0], p[1]) for p in [
		(Point(350, 0), RED), (Point(350, 400), BLUE)]]

		self.characters['zones'] = self.startingZones + self.defenseBuffZones + \
		    self.loadingZones

		# self.background = Rectangle(Point(0, 0), self.width, self.height, 0)

		self.viewer = rendering.Viewer(self.width, self.height)

		boundary = rendering.PolyLine([(1, 0), (1, 499), (800, 499), (800, 0)], True)
		boundary.set_color(COLOR_BLACK[0], COLOR_BLACK[1], COLOR_BLACK[2])
		self.viewer.add_geom(boundary)

		for char in self.characters['obstacles'] + self.characters['zones']:
			geoms = char.render()
			for geom in geoms:
				self.viewer.add_geom(geom)

	def state(self):
		return []

	def actables(self):
		return self.characters['robots'] + self.characters['bullets']

	def unpenetrables(self):
		return self.characters['robots'] + self.characters['obstacles']

	def impermissibles(self, robot):
		return list(filter(lambda r: not r == robot, self.characters['robots'])) \
		    + self.characters['obstacles'] \
		    + list(filter(lambda z: not z.permissble(robot.team), self.loadingZones))

	"""
	BUGGY. DON'T USE
	"""
	def direct_reachable_forward(self, robot, to):
		if floatEquals(robot.center.x, to.x) and floatEquals(robot.center.y, to.y):
			return True
		helper_rec = Rectangle(robot.bottom_left, robot.center.dis(to) + robot.width, \
		    robot.height, robot.angleTo(to))
		return not self.isObstructed(helper_rec, robot)

	# def direct_reachable_sideways

	def hasWinner(self):
		my_health, enemy_health = self.my_team.totalHealth(), self.enemy_team.totalHealth()
		if my_health == 0:
			return self.enemy_team
		if enemy_health == 0:
			return self.my_team
		if self.game_time == self.full_time:
			if my_health > enemy_health:
				return self.my_team
			if enemy_health > my_health:
				return self.enemy_team
			return "DRAW"

	# Moves the game 1 timestep defined by self.tau
	def step(self):
		# """
		# robot_action: a point(x, y) the robot will travel to
		# enemy_action: a point(x, y) the enemy robot will travel to
		# """

		winner = self.hasWinner()
		if winner:
			self.finished = True
			if winner == "DRAW":
				print('GAME OVER. RESULT: DRAW')
			else:
				print('GAME OVER. WINNER IS {0}.'.format(winner.name))
			self.close()
			return

		self.game_time += RobomasterEnv.tau

		if self.game_time % 3000 == 0:
			for z in self.defenseBuffZones + self.loadingZones:
				z.reset()

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
		self.viewer.close()
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

		for char in self.characters['robots'] + self.characters['bullets']:
			geoms = char.render()
			for geom in geoms:
				self.viewer.add_onetime(geom)

		return self.viewer.render(return_rgb_array = mode == 'rgb_array')

	def isObstructed(self, rec, robot):
		for ob in self.impermissibles(robot):
			if ob.intersects(rec):
				return True
		for v in robot.vertices:
			if not self.isLegal(v):
				return True
		return False

	# Returns the object blocking the line segment
	# Returns False if it's not blocked
	def isBlocked(self, seg, target=None):
		for block in self.unpenetrables():
			if block.blocks(seg):
				if block is target:
					continue
				return block
		return False

	def isLegal(self, point):
		return point.x >= 0 and point.x <= self.width and point.y >= 0 and point.y <= self.height

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None
