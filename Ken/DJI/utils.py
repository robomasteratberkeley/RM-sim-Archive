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

	def move_seg_by_angle(self, angle, dis):
		angle = toRadian(angle)
		to = self.move(dis * math.cos(angle), dis * math.sin(angle))
		return LineSegment(self, to)

	def diff(self, fr):
		return Vector(self.x - fr.x, self.y - fr.y)

	def floatEquals(self, other):
		return floatEquals(self.x, other.x) and floatEquals(self.y, other.y)

	def dis(self, to):
		return math.sqrt((self.x - to.x) ** 2 + (self.y - to.y) ** 2)

	def midpoint(self, other):
		return Point((self.x + other.x) / 2, (self.y + other.y) / 2)

	def four_split(self, other):
		mid = self.midpoint(other)
		return [self.midpoint(mid) + mid + mid.midpoint(other)]

	def toList(self):
		return [self.x, self.y]


"""
Contains vector functions for convenience
"""
class Vector(Point):

	def __init__(self, x, y):
		super().__init__(x, y)
		self.length = self.dis(Point(0, 0))

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


class LineSegment:

	def __init__(self, point_from, point_to):
		self.point_from = point_from
		self.point_to = point_to
		self.move_vec = point_to.diff(point_from)
		if point_from.x == point_to.x:
			self.slope = float('inf')
		else:
			self.slope = (point_to.y - point_from.y) / (point_to.x - point_from.x)

	def y_at(self, x):
		if self.slope == float('inf'):
			return float('inf')
		return self.point_from.y + (x - self.point_from.x) * self.slope

	def length(self):
		return self.point_from.dis(self.point_to)

class Team:

	def __init__(self, name):
		self.name = name
		self.robots = []
		self.enemy = None

		if self.name == "BLUE":
			self.dark_color = COLOR_DARKBLUE
			self.color = COLOR_BLUE
		else:
			self.dark_color = COLOR_DARKRED
			self.color = COLOR_RED

	def addRobot(self, robot):
		self.robots.append(robot)

	def addDefenseBuff(self, time):
		for r in self.robots:
			r.addDefenseBuff(time)

	def totalHealth(self):
		return sum([r.health for r in self.robots])


def toRadian(deg):
	return deg / 180 * math.pi

def toDegree(rad):
	return rad * 180 / math.pi

def floatEquals(a, b, error_threshold=0.01):
	return abs(a - b) < error_threshold


COLOR_BLUE = (0, 0, 1)
COLOR_DARKBLUE = (0, 0, 0.5)
COLOR_RED = (1, 0, 0)
COLOR_DARKRED = (0.5, 0, 0)
COLOR_GREEN = (0, 1, 0)
COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (1, 1, 1)
