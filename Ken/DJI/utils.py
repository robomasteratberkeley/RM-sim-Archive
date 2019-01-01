import math

"""
Implements a point
"""
class Point:

	def __init__(self, x, y):
		self.x = x
		self.y = y

	def move(self, dx, dy):
		return Point(self.x + dx, self.y + dy)

	def diff(self, fr):
		return Vector(self.x - fr.x, self.y - fr.y)

	def midpoint(self, other):
		return Point((self.x + other.x) / 2, (self.y + other.y) / 2)

	def toList(self):
		return [self.x, self.y]


"""
Contains vector functions for convenience
"""
class Vector(Point):

	def __init__(self, x, y):
		super().__init__(x, y)
		self.length = math.sqrt(x ** 2 + y ** 2)

	def dot(self, other):
		return self.x * other.x + self.y * other.y

	def scale(self, factor):
		return Vector(self.x * factor, self.y * factor)

	def normalize(self):
		if self.length == 0:
			return Vector(0, 0)
		return self.scale(1 / self.length)

	def project(self, other):
		return other.normalize().scale(self.dot(other) / other.length)

	def angle_radian(self):
		deg = math.atan(self.y / self.x)
		if self.x < 0:
			deg += math.pi
		if deg < 0:
			deg += 2 * math.pi
		return deg

class Team:

	def __init__(self, color, name):
		self.color = color
		self.name = name
		self.robots = []
		self.enemy = None

	def addRobot(self, robot):
		self.robots.append(robot)

	def addDefenseBuff(time):
		for r in robots:
			r.addDefenseBuff(time)

	def totalHealth(self):
		return sum([r.health for r in self.robots])


def toRadian(deg):
	return deg / 180 * math.pi

def toDegree(rad):
	return rad * 180 / math.pi
