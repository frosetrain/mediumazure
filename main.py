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
# side_color_sensor = ColorSensor(Port.C)
cage_motor = Motor(Port.C, Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 56, 160)
db.use_gyro(True)
# db.settings(straight_speed=425, straight_acceleration=1000, turn_rate=250, turn_acceleration=500)


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
            db.drive(425, difference * 0.4)


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
            db.drive(425, 0)
        else:
            db.drive(425, 0)


def detect():
    db.reset()
    averages = [0] * 6

    for i in range(6):
        slot_start = i * (31.9 + 52.15)
        slot_end = slot_start + 31.9
        while db.distance() < slot_start:
            db.drive(100, 0)
        total_s = 0
        total_v = 0
        tally = 0
        while db.distance() < slot_end:
            db.drive(100, 0)
            tally += 1
            hsv = left_color_sensor.hsv()
            total_s += hsv.s
            total_v += hsv.v
        averages[i] = (total_s + total_v) / tally

    print(averages)
    sorted_averages = sorted([(value, index) for index, value in enumerate(averages)], reverse=True)
    park_elements = [False] * 6
    for a in sorted_averages[0:2]:
        park_elements[a[1]] = True


# Go straight
# db.curve(110, 52)
# db.curve(110, -52)

cage_motor.run_target(200, 135)
db.curve(70, 90)
# exit()
# linetrack("stop", 50, 160)
db.straight(256)
db.turn(90)
db.straight(70)
cage_motor.run_target(200, 57)
db.straight(-130)
db.turn(-90)
linetrack("both", 100, 1050)
db.curve(130, 90)
cage_motor.run_target(200, 135)
db.straight(-130)
db.straight(100)

# Starter kit stuff
# db.curve(67, -90)
# cage_motor.run_target(360, 0)
# db.straight(-67)
# db.turn(-90)
# linetrack("green", 50, 120)
# db.straight(37)
# db.turn(-90)
# linetrack("left", 100, 450)
