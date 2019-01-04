
from Objects import *
from utils import *

"""
All strategies shall implement the abstract class Strategy

A strategy takes in an env state and returns an action

Note that users cannot create new instances of Strategy.
They use the CLASS OBJECT for operations
"""
class Strategy:

    def decide(self, robot):
        pass

    def name(self):
        pass


class Patrol(Strategy):

    key_points = [Point(0, 0)]

    def decide(self, robot):
        pass

    def name(self):
        return "PATROL"


class DoNothing(Strategy):

    def decide(self, robot):
        return None

    def name(self):
        return "DO NOTHING"


class SpinAndFire(Strategy):

    def decide(self, robot):
        actions = []
        if robot.angle % 90 == 0:
            actions.append(Fire())
        if robot.angle < 180:
            return actions + [Rotate(180)]
        if robot.angle >= 180 and robot.angle < 270:
            return actions + [Rotate(270)]
        return actions + [Rotate(0)]

    def name(self):
        return "SPIN&FIRE"


class AimAndFire(Strategy):

    def decide(self, robot):
        target = robot.team.enemy.robots[0]
        if robot.angleTo(target) == robot.angle:
            return Fire()
        return Aim(target)


"""
An Action is a command to the robot

It changes the state of the robot upon resolve(robot)
"""
class Action:

    def resolve(self, robot):
        pass


class Rotate(Action):

    def __init__(self, angle):
        self.angle = angle

    def resolve(self, robot):
        if robot.angle == self.angle:
        	return
        diff = self.angle - robot.angle
        min_dis = min(diff, diff + 360, diff - 360, key = lambda n: abs(n))
        if min_dis > 0:
        	change = min(robot.max_rotation_speed, min_dis)
        else:
        	change = max(-robot.max_rotation_speed, min_dis)

        helper_rec = Rectangle(robot.center, robot.width / 2, robot.height / 2, robot.angle + 180 + change)

        return Rectangle(helper_rec.vertices[2], robot.width, robot.height, robot.angle + change)


class Aim(Action):

    def __init__(self, target):
        self.target = target

    def resolve(self, robot):
        return Rotate(robot.angleTo(self.target)).resolve(robot)


class Fire(Action):

    def resolve(self, robot):
        if robot.bullet > 0:
            robot.env.characters['bullets'].append(Bullet(robot.gun.center, robot.angle + robot.gun_angle, robot.team, robot.env))
            robot.bullet -= 1
