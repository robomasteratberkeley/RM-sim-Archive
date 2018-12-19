"""
Robot and obstacle classes for DJI Robomaster AI Challenge
"""

import numpy as np
import heapq
import cv2

class Robot:
	def __init__(self, env, enemy=False):
		self.health = 100
		self.width = 30.0
		self.length = 50.0
		self.gun_width = self.width / 2
		self.gun_length = self.length
		self.gun_angle = 0

		self.max_x_speed = 150
		self.max_y_speed = 100

		if enemy:
			self.x, self.y = env.width - self.width / 2, env.height - self.length / 2
		else:
			self.x, self.y = self.width / 2, self.length / 2

		self.angle = 90.0
		# need to adjust functions to account for changes in robot orientation (angle)

	def set_speed(self, x_coord, y_coord):
		x_speed = (x_coord - self.x)/10 * self.max_x_speed
		y_speed = (y_coord - self.y)/10 * self.max_y_speed
		return [min(x_speed, x_speed/abs(x_speed)*self.max_x_speed, key=lambda speed: abs(speed)), min(y_speed, y_speed/abs(y_speed)*self.max_y_speed, key=lambda speed: abs(speed))]

	def move(self, other_robot, env, x_coord, y_coord, obstacles, tau):
		angle = self.angle * np.pi / 180

		x_speed, y_speed = self.set_speed(x_coord, y_coord)

		new_x = self.x + (x_speed*np.sin(angle) + y_speed*np.cos(angle))*tau
		new_y = self.y + (-x_speed*np.cos(angle) + y_speed*np.sin(angle))*tau

		# Check for collisions

		# Outside Bounds
		if new_x < self.width / 2:
			new_x + self.width / 2
		elif new_x + self.width / 2 > env.width:
			new_x = env.width - self.width / 2

		if new_y < self.length / 2:
			new_y = self.length / 2
		elif new_y + self.length / 2 > env.height:
			new_y = env.height - self.length / 2

		# Obstacles
		for obstacle in obstacles:
			l = obstacle.l - self.width / 2
			r = obstacle.r + self.width / 2
			b = obstacle.b - self.length / 2
			t = obstacle.t + self.length / 2
			if l < new_x < r and b < new_y < t:
				if self.x <= l:
					new_x = l
				elif self.x >= r:
					new_x = r
				if self.y <= b:
					new_y = b
				elif self.y >= t:
					new_y = t

		# Other robot
		l = other_robot.x - other_robot.width / 2 - self.width / 2
		r = other_robot.x + other_robot.width / 2 + self.width / 2
		b = other_robot.y - other_robot.length / 2 - self.length / 2
		t = other_robot.y + other_robot.length / 2 + self.length / 2

		if l < new_x < r and b < new_y < t:
			if self.x <= l:
				new_x = l
			elif self.x >= r:
				new_x = r
			if self.y <= b:
				new_y = b
			elif self.y >= t:
				new_y = t

		self.x, self.y = new_x, new_y

	def aim(self, other, obstacles):
		if self.x == other.x:
			if self.y > other.y:
				gun_angle = -np.pi / 2
			else:
				gun_angle = np.pi / 2
		elif self.x < other.x:
			gun_angle = np.arctan((self.y - other.y) / (self.x - other.x))
		else:
			gun_angle = np.arctan((self.y - other.y) / (self.x - other.x)) + np.pi

		# Check for visual obstruction
		x, y = self.x, self.y
		step = 20

		obstructed = False
		while abs(x - self.x) < abs(other.x - self.x):
			for obstacle in obstacles:
				if obstacle.surrounds(x, y):
					obstructed = True # Return and skips the step of updating self.gun_angle
			x += step * np.cos(gun_angle)
			y += step * np.sin(gun_angle)
		
		if not obstructed:
			self.gun_angle = (gun_angle * 180 / np.pi - 90) % 360

	def random_action(self):
		return (np.random.random_sample(2,))*[800, 500]

class Obstacle:
	def __init__(self, l, r, t, b):
		self.l = l
		self.r = r
		self.t = t
		self.b = b

	def surrounds(self, x, y):
		if self.l < x < self.r and self.b < y < self.t:
			return True
		else:
			return False

