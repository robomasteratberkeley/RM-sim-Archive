"""
Describes all objects present in the challenge field
"""

import numpy as np
import math
import heapq
import cv2
import rendering
import keyboard

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
		self.sides = self.getSides()
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

	def getSides(self):
		return [LineSegment(self.vertices[i], self.vertices[(i + 1) % 4]) for i in range(4)]

	def getCenter(self):
		return self.bottom_left.midpoint(self.vertices[2])

	def setAngle(self, deg):
		self.angle = deg % 360
		self.angle_radian = deg / 180 * math.pi

	"""
	Check if a point is contained by self. Uses some messy linalg. Please Suggest
	better implementation if possible.
	"""
	def contains(self, point):
		error_threshold = 0.01

		width_vec = self.vertices[1].diff(self.bottom_left)
		height_vec = self.vertices[3].diff(self.bottom_left)

		goal_vec = point.diff(self.bottom_left)
		width_proj = goal_vec.project(width_vec)
		height_proj = goal_vec.project(height_vec)

		return width_proj.length - self.width < error_threshold and \
	       height_proj.length - self.height < error_threshold and \
	       width_proj.dot(width_vec) >= 0 and \
		   height_proj.dot(height_vec) >= 0

	def contains_any(self, points):
		for p in points:
			if self.contains(p):
				return True
		return False

	def contains_all(self, points):
		for p in points:
			if not self.contains(p):
				return False
		return True

	"""
	Checks if two rectangles intersect
	IT DOESN'T CONSIDER SOME CASES which I don't think are necessary for our app
	"""
	def intersects(self, other):
		for side in other.sides:
			if self.blocks(side):
				return True
		return False

	def blocks(self, seg):
		return self.contains(seg.point_to) or self.contains(seg.point_from)

	def angleTo(self, point):
		if self.center.x == point.x:
			if self.center.y > point.y:
				return 270
			return 90
		return toDegree(point.diff(self.center).angle_radian())

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
		self.left = bottom_left.x
		self.right = top_right.x
		self.top = top_right.y
		self.bottom = bottom_left.y
		return [bottom_left, bottom_right, top_right, top_left]

	def contains(self, point):
		x, y = point.x, point.y
		return self.left <= x and self.bottom <= y and self.right >= x and self.top >= y

	def blocks(self, seg):
		point_from, point_to = seg.point_from, seg.point_to
		if self.contains(point_from) or self.contains(point_to):
			return True
		if (point_from.x < self.left and point_to.x < self.left) or \
		   (point_from.x > self.right and point_to.x > self.right) or \
		   (point_from.y < self.bottom and point_to.y < self.bottom) or \
		   (point_from.y > self.top and point_to.y > self.top):
		   return False

		left_y, right_y = seg.y_at(self.left), seg.y_at(self.right)

		return not (left_y > self.top and right_y > self.top) and \
		       not (left_y < self.bottom and right_y < self.bottom)

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
		self.loadingPoint = self.center

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
		return floatEquals(self.loadingPoint.x, robot.center.x, self.tolerance_radius) and \
		    floatEquals(self.loadingPoint.y, robot.center.y, self.tolerance_radius)

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
			print(self.team.name + " team has activated defense buff!")
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
	range = 300

	def __init__(self, point, dir, env):
		self.delay = 0  # Models the delay from firing decision to bullet actually flying
		self.speed = 25
		self.point = point
		self.dir = dir / 180 * math.pi
		self.env = env
		self.active = True

	def act(self):
		if not self.active:
			return
		if self.delay > 0:
			self.delay -= 1
			return
		move_point = self.point.move(math.cos(self.dir) * self.speed, math.sin(self.dir) * self.speed)
		move_seg = LineSegment(self.point, move_point)

		blocker = self.env.isBlocked(move_seg)
		if blocker:
			self.destruct()
			if blocker.isRobot():
				blocker.reduceHealth(self.damage)
		elif self.env.isLegal(move_point):
			self.point = move_point
			self.range -= self.speed
			if self.range <= 0:
				self.destruct()
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
	range = 300 # More on this later

	max_forward_speed = 1.5
	max_sideway_speed = 1
	max_rotation_speed = 1.5
	max_gun_rotation_speed = 6
	max_cooldown = 11

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
		self.cooldown = 0
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

	def getEnemy(self):
		enemy = self.team.enemy
		if enemy.robots[0].health == 0:
			return enemy.robots[1]
		return enemy.robots[0]

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
			self.defenseBuffTimer = max(0, self.defenseBuffTimer - 1)
			self.freezeTimer = max(0, self.freezeTimer - 1)
			self.cooldown = max(0, self.cooldown - 1)
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
						return action.resolve(self)
					for action_part in action:
						action.resolve(self)

	def setPosition(self, rec):
		Rectangle.__init__(self, rec.bottom_left, rec.width, rec.height, rec.angle)
		self.gun = self.getGun()

	def getGun(self):
		bottom_left = self.vertices[1].midpoint(self.center).midpoint(self.center)
		return Rectangle(bottom_left, self.gun_length, self.gun_width, self.angle + self.gun_angle)

	def fireLine(self):
		return self.gun.center.move_seg_by_angle(self.angle, 1000)

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
		return Attack


class ManualControlRobot(Robot):

	def __init__(self, controls, env, team, bottom_left, angle=0):
		super().__init__(env, team, bottom_left, angle)
		self.controls = controls

	def getStrategy(self):
		return Manual(self.controls)
