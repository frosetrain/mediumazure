from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import GyroDriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
left_color_sensor = ColorSensor(Port.D)
right_color_sensor = ColorSensor(Port.C)
ultrasonic = UltrasonicSensor(Port.E)
helikopter = Motor(Port.F)
db = GyroDriveBase(left_motor, right_motor, 56, 16 * 8 + 2)
db.settings(
    straight_speed=400,
    straight_acceleration=1000,
    turn_rate=200,
    turn_acceleration=800,
)


def linetrack(junction_type, stretch):
    slow = True
    db.reset()
    while True:
        if slow and db.distance() > 100:
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
        if hit:
            db.stop()
            db.straight(96)
            break
        difference = left_reflection - right_reflection
        if slow:
            db.drive(200, difference / 6)
        else:
            db.drive(400, difference / 6)


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
            db.stop()
            db.straight(96)
            break
        if slow:
            db.drive(200, 0)
        else:
            db.drive(400, 0)


while True:
    linetrack("both", 1200)
    db.turn(-120)
    db.straight(575)
    db.turn(-60)
    drivetoline(700)
    db.turn(-90)
    linetrack("green", 500)
    db.straight(30)
    db.turn(-90)


# db.turn(360)
