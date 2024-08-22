from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
left_color_sensor = ColorSensor(Port.A)
right_color_sensor = ColorSensor(Port.E)
stick_motor = Motor(Port.D, Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 62.4, 144)
# db.use_gyro(True)
db.settings(straight_speed=469, straight_acceleration=1500, turn_acceleration=1000, turn_rate=250)


def linetrack(junction_type, start, stretch):
    slow = True
    db.reset()
    while True:
        if slow and db.distance() > start:
            slow = False
        if not slow and db.distance() > stretch:
            slow = True
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        print(left_reflection, right_reflection)
        if junction_type == "left":
            hit = left_reflection < 25 and right_reflection > 60
        elif junction_type == "both":
            hit = left_reflection < 25 and right_reflection < 25
        elif junction_type == "right":
            hit = left_reflection > 60 and right_reflection < 25
        elif junction_type == "green":
            hit = left_color_sensor.color() == Color.GREEN and right_color_sensor.color() == Color.GREEN
        elif junction_type == "stop":
            hit = True
        if hit and db.distance() > stretch:
            db.straight(96, then=Stop.BRAKE)
            break
        difference = left_reflection - right_reflection
        if slow:
            db.drive(200, difference * 0.6)
        else:
            db.drive(469, difference * 0.4)


def drivetoline(stretch):
    slow = False
    db.reset()
    while True:
        if not slow and db.distance() > stretch:
            slow = True
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        print(left_reflection, right_reflection)
        if left_reflection < 25 and right_reflection < 25:
            db.straight(96, then=Stop.BRAKE)
            break
        if slow:
            db.drive(469, 0)
        else:
            db.drive(469, 0)


stick_motor.run_target(200, -51)
db.curve(110, 52)
db.curve(110, -52)
linetrack("stop", 0, 150)
db.curve(67, -90)
stick_motor.run_target(360, 0)
db.straight(-67)
db.turn(-90)
linetrack("green", 50, 120)
db.straight(37)
db.turn(-90)
linetrack("left", 100, 450)
