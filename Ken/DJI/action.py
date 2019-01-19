
from Objects import *
from utils import *

"""
An Action is a command to the robot

It changes the state of the robot upon resolve
"""
class Action:

    def frozenOk():
        return False

    # Returns True if the Action is successfully resolved
    # Returns False if it didn't have any effect
    def resolve(self, robot):
        if robot.freezeTimer > 0 and not self.frozenOk():
            return False
        result_rec = self.simple_resolve(robot)
        if not result_rec:
            return True
        if robot.env.isObstructed(result_rec, robot):
        	return False
        robot.setPosition(result_rec)
        return True


class Step(Action):

    def __init__(self, robot_angle, dis):
        self.angle += robot_angle
        self.dx = math.cos(toRadian(self.angle)) * dis
        self.dy = math.sin(toRadian(self.angle)) * dis

    def simple_resolve(self, robot):
        return Rectangle(robot.bottom_left.move(self.dx, self.dy), robot.width, robot.height, robot.angle)


class StepForward(Step):
    angle = 0

class StepBackward(Step):
    angle = 180

class StepLeft(Step):
    angle = 90

class StepRight(Step):
    angle = 270


class Rotate(Action):

    def frozenOk(self):
        return True

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):
        if floatEquals(robot.angle, self.angle):
        	return
        diff = self.angle - robot.angle
        min_dis = min(diff, diff + 360, diff - 360, key = lambda n: abs(n))
        if min_dis > 0:
            angle = min(robot.max_rotation_speed, min_dis)
            action = RotateLeft(angle)
        else:
            angle = min(robot.max_rotation_speed, -min_dis)
            action = RotateRight(angle)

        return action.simple_resolve(robot)


class RotateLeft(Action):

    def frozenOk(self):
        return True

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):

        helper_rec = Rectangle(robot.center, robot.width / 2, robot.height / 2, robot.angle + 180 + self.angle)
        return Rectangle(helper_rec.vertices[2], robot.width, robot.height, robot.angle + self.angle)


class RotateRight(Action):

    def frozenOk(self):
        return True

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):

        helper_rec = Rectangle(robot.center, robot.width / 2, robot.height / 2, robot.angle + 180 - self.angle)
        return Rectangle(helper_rec.vertices[2], robot.width, robot.height, robot.angle - self.angle)


class RefillCommand(Action):

    def resolve(self, robot):
        print(robot.team.name + " team issued reload command!")
        robot.team.loadingZone.load(robot)


class RotateGunLeft(Action):

    def frozenOk(self):
        return True

    def resolve(self, robot):
        robot.gun_angle = (robot.gun_angle - robot.max_gun_rotation_speed) % 360


class RotateGunRight(Action):

    def frozenOk(self):
        return True

    def resolve(self, robot):
        robot.gun_angle = (robot.gun_angle + robot.max_gun_rotation_speed) % 360


class Aim(Action):

    def frozenOk(self):
        return True

    def __init__(self, target_point):
        self.target_point = target_point

    def simple_resolve(self, robot):
        return Rotate(robot.angleTo(self.target_point)).simple_resolve(robot)


class Fire(Action):

    def frozenOk(self):
        return True

    def resolve(self, robot):
        if robot.bullet > 0 and robot.cooldown == 0:
            robot.env.characters['bullets'].append(Bullet(robot.gun.center, robot.angle + robot.gun_angle, robot.env))
            robot.bullet -= 1
            robot.cooldown = robot.max_cooldown



class Move(Action):

    def __init__(self, target_point):
        self.target_point = target_point

    def simple_resolve(self, robot):
        if robot.center.floatEquals(self.target_point):
            return
        if floatEquals(robot.angleTo(self.target_point), robot.angle):
            return StepForward(robot.angle, min(robot.center.dis(self.target_point), \
                robot.max_forward_speed)).simple_resolve(robot)
        return Aim(self.target_point).simple_resolve(robot)
