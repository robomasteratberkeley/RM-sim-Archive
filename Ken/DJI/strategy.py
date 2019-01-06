
from Objects import *
from utils import *

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
                print(robot.env.isBlocked(fire_line, self.target_robot))
                return
            if fire_line.length() > robot.range:
                return MoveFrontAndBack(fire_line.length())
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


"""
An Action is a command to the robot

It changes the state of the robot upon resolve(robot)
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


class Rotate(Action):

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):
        if floatEquals(robot.angle, self.angle):
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

    def __init__(self, target_point):
        self.target_point = target_point

    def simple_resolve(self, robot):
        return Rotate(robot.angleTo(self.target_point)).simple_resolve(robot)


class Fire(Action):

    def frozenOk(self):
        return True

    def simple_resolve(self, robot):
        if robot.bullet > 0:
            robot.env.characters['bullets'].append(Bullet(robot.gun.center, robot.angle + robot.gun_angle, robot.env))
            robot.bullet -= 1


class RefillCommand(Action):

    def simple_resolve(self, robot):
        print(robot.team.name + " team issued reload command!")
        robot.team.loadingZone.load(robot)


class Move(Action):

    def __init__(self, target_point):
        self.target_point = target_point

    def simple_resolve(self, robot):
        if robot.center.floatEquals(self.target_point):
            return
        if robot.env.direct_reachable(robot, self.target_point):
            if floatEquals(robot.angleTo(self.target_point), robot.angle):
                return MoveFrontAndBack(robot.center.dis(self.target_point)).resolve(robot)
            return Aim(self.target_point).simple_resolve(robot)


class MoveFrontAndBack(Action):

    def __init__(self, dis):
        self.dis = dis

    def resolve(self, robot):
        speed = robot.max_forward_speed
        dis = int(self.dis)
        if dis > speed:
            dis = speed
        for i in range(dis):
            if not CreepForward().resolve(robot):
                return i


class CreepForward(Action):

    def simple_resolve(self, robot):
        return Rectangle(robot.bottom_left.move(math.cos(toRadian(robot.angle)), \
            math.sin(toRadian(robot.angle))), robot.width, robot.height, robot.angle)
