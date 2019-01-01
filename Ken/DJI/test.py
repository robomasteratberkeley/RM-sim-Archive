from Objects import *

rec = Rectangle(Point(0, 0), 80, 80, 45)
rec2 = Rectangle(Point(60, 60), 30, 30, 180)

print(rec.intersects(rec2))
