"""
Describes all objects present in the challenge field
"""

import numpy as np
import math
import heapq
import cv2
import rendering

from utils import *
from strategy import *

"""
Every object is an implementation of the abstract class Character
"""
class Character:

	color = COLOR_BLACK

	def act(self, env):
		pass

	def render(self):
		pass

	def reset(self):
		pass

	def isRobot(self):
		return False

"""
All rectangular objects are described by class Rectangle

defined by the rectangle's center, width(x-span), height(y-span) and angle in degrees
"""
class Rectangle(Character):

	def __init__(self, bottom_left, width, height, angle=0):
		self.bottom_left = bottom_left
		self.width = width
		self.height = height
		self.setAngle(angle)
		self.angle_radian = angle / 180 * math.pi
		self.vertices = self.getVertices()
		self.center = self.getCenter()

	def getVertices(self):
		width_dx = math.cos(self.angle_radian) * self.width
		width_dy = math.sin(self.angle_radian) * self.width

		height_dx = -math.sin(self.angle_radian) * self.height
		height_dy = math.cos(self.angle_radian) * self.height

		bottom_right = self.bottom_left.move(width_dx, width_dy)
		top_left = self.bottom_left.move(height_dx, height_dy)
		top_right = bottom_right.move(height_dx, height_dy)

		return [self.bottom_left, bottom_right, top_right, top_left]

	def getCenter(self):
		return self.bottom_left.midpoint(self.vertices[2])

	def setAngle(self, deg):
		if deg < 0:
			return self.setAngle(deg + 360)
		if deg >= 360:
			return self.setAngle(deg - 360)
		self.angle = deg
		self.angle_radian = deg / 180 * math.pi

	"""
	Check if a point is contained by self. Uses some messy linalg. Please Suggest
	better implementation if possible.
	"""
	def contains(self, point):
		return self.contains_any([point])

	def contains_list(self, points):
		error_threshold = 0.01

		width_vec = self.vertices[1].diff(self.bottom_left)
		height_vec = self.vertices[3].diff(self.bottom_left)

		def contains_point(point):
			goal_vec = point.diff(self.bottom_left)
			width_proj = goal_vec.project(width_vec)
			height_proj = goal_vec.project(height_vec)

			return width_proj.length - self.width < error_threshold and \
		       height_proj.length - self.height < error_threshold and \
		       width_proj.dot(width_vec) >= 0 and \
			   height_proj.dot(height_vec) >= 0

		return [contains_point(p) for p in points]

	def contains_any(self, points):
		return any(self.contains_list(points))

	def contains_all(self, points):
		return all(self.contains_list(points))

	"""
	Checks if two rectangles intersect
	IT DOESN'T CONSIDER SOME CASES which I don't think are necessary for our app
	"""
	def intersects(self, other):
		return self.contains_any(other.vertices) or other.contains_any(self.vertices)

	def blocks(self, point, vec):
		return self.contains(point)

	def angleTo(self, other):
		my_center, their_center = self.center, other.center
		return toDegree(their_center.diff(my_center).angle_radian())

	"Renders the rectangle depending on type"
	def render(self, color=None):
		rec = rendering.FilledPolygon([p.toList() for p in self.vertices])
		if not color:
			color = self.color
		rec.set_color(color[0], color[1], color[2])
		return [rec]


"""
Type of rectangle that never rotates.
Has implementation of certain methods that are more efficient
"""
class uprightRectangle(Rectangle):

	def getVertices(self):
		bottom_left = self.bottom_left
		bottom_right = bottom_left.move(self.width, 0)
		top_right = bottom_right.move(0, self.height)
		top_left = bottom_left.move(0, self.height)
		return [bottom_left, bottom_right, top_right, top_left]

	def contains(self, point):
		bottom_left, top_right = self.bottom_left, self.vertices[2]
		x, y = point.x, point.y
		return bottom_left.x <= x and bottom_left.y <= y and \
		    top_right.x >= x and top_right.y >= y

"""
Impermissible and inpenetrable obstacles are described by class Obstacle
"""
class Obstacle(uprightRectangle):

	def permissible(self, team):
		return False

	def penetrable(self):
		return False


"""
Permissble and penetrable areas in the field are described by class Zone
"""
class Zone(uprightRectangle):

	sidelength = 100

	def __init__(self, bottom_left, team):
		super().__init__(bottom_left, 100, 100)
		self.team = team
		self.color = team.dark_color

	def penetrable(self):
		return True

	def permissble(self, team):
		return True

	def render(self):
		return super().render() + Rectangle(self.bottom_left.move(8, 8), \
		    self.width - 16, self.height - 16).render(COLOR_WHITE)


"""
Loading zones that provide 17mm bullets
"""
class LoadingZone(Zone):

	def __init__(self, bottom_left, team):
		super().__init__(bottom_left, team)
		team.loadingZone = self

	fills = 2
	tolerance_radius = 3

	"""
	Enemy loading zone is modeled as impermissible
	"""
	def permissble(self, team):
		return self.team == team

	"""
	Checks if the robot is aligned with the bullet supply machinary
	"""
	def aligned(self, robot):
		return floatEquals(self.center.x, robot.center.x, self.tolerance_radius) and \
		    floatEquals(self.center.y, robot.center.y, self.tolerance_radius)

	def load(self, robot):
		if self.fills <= 0:
			print("Team " + robot.team.name + " has no more reloads available.")
			return
		if self.aligned(robot):
			print("Reload successful on robot " + str(robot.id))
			robot.load(100)
			robot.freeze(10)
		self.fills -= 1

	def reset(self):
		self.fills = 2

	def render(self):
		circle = rendering.Circle(self.center, 12)
		circle.set_color(self.color[0], self.color[1], self.color[2])
		return super().render() + [circle]


"""
Buff zone that boosts defense for one team
"""
class DefenseBuffZone(Zone):

	# color =

	active = True

	def __init__(self, bottom_left, team):
		super().__init__(bottom_left, team)
		self.d_helper = Rectangle(self.center.move(0, -15), \
		    15 * 1.414, 15 * 1.414, 45)
		team.defenseBuffZone = self
		self.touch_rec = [0, 0, 0, 0]

	def touch(self, robot):
		self.touch_rec[robot.id] += 1
		if self.touch_rec[robot.id] == 500:
			self.activate()

	def activate(self):
		if self.active:
			self.team.addDefenseBuff(30)
			self.active = False

	def reset(self):
		self.active = True

	def render(self):
		return super().render() + self.d_helper.render(self.color)


class StartingZone(Zone):

	pass


"""
Describes a bullet. Currently modeled as having constant speed and no volume
"""
class Bullet:

	damage = 50
	range = float('inf')

	def __init__(self, point, dir, team, env):
		self.delay = 0  # Models the delay from firing decision to bullet actually flying
		self.speed = 25
		self.travelled = 0
		self.point = point
		self.dir = dir / 180 * math.pi
		self.team = team
		self.env = env
		self.active = True

	def act(self):
		if not self.active:
			return
		if self.delay > 0:
			self.delay -= 1
			return
		move_point = self.point.move(math.cos(self.dir) * self.speed, math.sin(self.dir) * self.speed)
		move_vec = move_point.diff(self.point)
		for block in self.env.unpenetrables():
			if block.blocks(move_point, move_vec):
				self.destruct()
				if block.isRobot():
					block.reduceHealth(self.damage)
				return

		if self.env.isLegal(move_point):
			self.point = move_point
		else:
			self.destruct()

	"""
	SUBJECT TO CHANGE!
	"""
	def destruct(self):
		self.env.characters['bullets'] = list(filter(lambda b: not b == self, self.env.characters['bullets']))
		self.active = False

	def render(self):
		return [rendering.Circle(self.point, 2)]

"""
The robot object -
Currently modeled as a Rectangle object by assumption that gun has negligible chance of blocking a bullet

To modify strategy, extend the class and override the `getStrategy` method
"""
class Robot(Rectangle):

	width = 50.0
	height = 30.0
	health = 2000
	gun_width = height / 4
	gun_length = width
	range = float('inf') # More on this later

	max_forward_speed = 15
	max_sideway_speed = 10
	max_rotation_speed = 1.5

	def __init__(self, env, team, bottom_left, angle=0):
		self.gun_angle = 0
		self.env = env

		self.team = team
		self.color = team.color
		team.addRobot(self)
		self.defenseBuffTimer, self.freezeTimer = 0, 0
		super().__init__(bottom_left, Robot.width, Robot.height, angle)
		self.gun = self.getGun()
		self.heat = 0
		self.bullet = 0

	def render(self):
		if self.alive():
			return super().render() + Rectangle.render(self.gun, self.color)
		return super().render()

	def alive(self):
		return self.health > 0

	def isRobot(self):
		return True

	def load(self, num):
		self.bullet += num

	def freeze(self, num):
		self.freezeTimer += num

	def hasDefenseBuff(self):
		return self.defenseBuffTimer > 0

	def addDefenseBuff(self, time):
		self.defenseBuffTimer = time * 100

	"""
	Determine a strategy based on information in self.env
	"""
	def getStrategy(self):
		pass

	"""
	The function evoked by Environment each turn
	First decide on an strategy, then execute it
	"""
	def act(self):
		if self.alive():
			for z in self.env.defenseBuffZones:
				if z.contains_all(self.vertices):
					z.touch(self)
			if self.freezeTimer > 0:
				self.freezeTimer -= 1
				return
			strategy = self.getStrategy()
			if strategy:
				action = strategy.decide(self)
				if action:
					if not type(action) == list:
						return self.execute(action)
					for action_part in action:
						self.execute(action_part)
		self.defenseBuffTimer = max(0, self.defenseBuffTimer - 1)
		self.freezeTimer = max(0, self.freezeTimer - 1)

	def execute(self, action):
		if self.freezeTimer > 0 and not action.frozenOk():
			return
		result_rec = action.resolve(self)
		if result_rec == None or self.env.isObstructed(result_rec, self):
			return
		self.setPosition(result_rec)

	def setPosition(self, rec):
		super().__init__(rec.bottom_left, rec.width, rec.height, rec.angle)
		self.gun = self.getGun()

	def getGun(self):
		bottom_left = self.vertices[1].midpoint(self.bottom_left).midpoint(self.center).midpoint(self.center)
		return Rectangle(bottom_left, self.gun_length, self.gun_width, self.angle)

	def reduceHealth(self, amount):
		if self.alive():
			if self.hasDefenseBuff():
				amount /= 2
			self.health = max(0, self.health - amount)


class DummyRobot(Robot):

	health = 10000

	def getStrategy(self):
		return DoNothing


class CrazyRobot(Robot):

	def getStrategy(self):
		return SpinAndFire


class AttackRobot(Robot):

	def getStrategy(self):
		target = self.team.enemy.robots[0]
		return AimAndFire(target)
