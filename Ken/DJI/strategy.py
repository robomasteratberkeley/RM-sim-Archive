
from Objects import *
from utils import *
from action import *
import keyboard

"""
All strategies shall implement the abstract class Strategy

A strategy takes in an env state and returns an action

Note that users cannot create new instances of some Strategies.
They use the CLASS OBJECT for operations
"""
class Strategy:

    def decide(robot):
        pass

    def name():
        pass


class Patrol(Strategy):

    key_points = [Point(0, 0)]

    def decide(robot):
        pass

    def name():
        return "PATROL"


class DoNothing(Strategy):

    def decide(robot):
        return None

    def name():
        return "DO NOTHING"


class SpinAndFire(Strategy):

    def decide(robot):
        actions = []
        if robot.angle % 90 == 0:
            actions.append(Fire())
        if robot.angle < 180:
            return actions + [Rotate(180)]
        if robot.angle >= 180 and robot.angle < 270:
            return actions + [Rotate(270)]
        return actions + [Rotate(0)]

    def name():
        return "SPIN&FIRE"


class AimAndFire(Strategy):

    def __init__(self, target_robot):
        self.target_robot = target_robot

    def decide(self, robot):
        if floatEquals(robot.angleTo(self.target_robot.center), robot.angle):
            fire_line = LineSegment(robot.gun.center, self.target_robot.center)
            if robot.env.isBlocked(fire_line, self.target_robot):
                return
            if fire_line.length() > robot.range:
                return StepForward(robot.angle, min(robot.center.dis(self.target_robot.center), \
                    robot.max_forward_speed))
            return Fire()
        return Aim(self.target_robot.center)


class Attack(Strategy):

    def decide(robot):
        enemy = robot.getEnemy()
        loader = robot.team.loadingZone
        if robot.bullet > 0:
            return AimAndFire(enemy).decide(robot)
        elif loader.aligned(robot):
            if loader.fills > 0:
                return RefillCommand()
            return None
        else:
            return Move(loader.loadingPoint)


class Manual(Strategy):

    def __init__(self, controls):
        self.controls = controls
        self.left = controls[0]
        self.down = controls[1]
        self.right = controls[2]
        self.up = controls[3]
        self.turnleft = controls[4]
        self.turnright = controls[5]
        self.fire = controls[6]
        self.refill = controls[7]

    def decide(self, robot):
        if keyboard.is_pressed(self.turnleft):
            return RotateLeft(robot.max_rotation_speed)
        if keyboard.is_pressed(self.turnright):
            return RotateRight(robot.max_rotation_speed)
        if keyboard.is_pressed(self.up):
            return StepForward(robot.angle, robot.max_forward_speed)
        if keyboard.is_pressed(self.down):
            return StepBackward(robot.angle, robot.max_forward_speed)
        if keyboard.is_pressed(self.left):
            return StepLeft(robot.angle, robot.max_forward_speed)
        if keyboard.is_pressed(self.right):
            return StepRight(robot.angle, robot.max_forward_speed)
        if keyboard.is_pressed(self.fire):
            return Fire()
        if keyboard.is_pressed(self.refill):
            return RefillCommand()
        return None
