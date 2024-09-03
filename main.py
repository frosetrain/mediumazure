"""Medium Azure, RMJ24_292_01."""

# ruff: noqa: ERA001, T201

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch

FAST_SPEED = 200
SLOW_SPEED = 100
BLACK_THRESHOLD = 25
WHITE_THRESHOLD = 60
LIGHT_SENSOR_X = 128

hub = PrimeHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
left_color_sensor = ColorSensor(Port.E)
right_color_sensor = ColorSensor(Port.A)
side_color_sensor = ColorSensor(Port.D)
cage_motor = Motor(Port.C, Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 56, 160)
db.use_gyro(use_gyro=True)
stopwatch = StopWatch()
# db.settings(straight_speed=FAST_SPEED, straight_acceleration=1000, turn_rate=250, turn_acceleration=500)


def linetrack(junction_type: str, start: int, stretch: int, *, move_forward: bool) -> None:  # noqa: C901
    """Line track.

    Args:
        junction_type (str): Left, both, right, green or stop.
        start (str): How far to drive before going quickly.
        stretch (int): How far to drive before slowing down and enabling detection.
        move_forward (bool): Whether to go forward to get axle on the line
    """
    slow = True
    db.reset()
    while True:
        if slow and db.distance() > start:
            slow = False
        if not slow and db.distance() > stretch:
            slow = True
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        # print(left_reflection, right_reflection)
        if junction_type == "left":
            hit = left_reflection < BLACK_THRESHOLD and right_reflection > WHITE_THRESHOLD
        elif junction_type == "both":
            hit = left_reflection < BLACK_THRESHOLD and right_reflection < BLACK_THRESHOLD
        elif junction_type == "right":
            hit = left_reflection > WHITE_THRESHOLD and right_reflection < BLACK_THRESHOLD
        elif junction_type == "green":
            hit = left_color_sensor.color() == Color.GREEN and right_color_sensor.color() == Color.GREEN
        elif junction_type == "stop":
            hit = True
        if hit and db.distance() >= stretch:
            if move_forward:
                db.straight(LIGHT_SENSOR_X, then=Stop.BRAKE)
            print("STOPPPP")
            db.brake()
            break
        difference = left_reflection - right_reflection
        if slow:
            db.drive(SLOW_SPEED, difference * 0.6)
        else:
            db.drive(FAST_SPEED, difference * 0.4)


def simple_linetrack(dist: int) -> None:
    """Simple linetrack to hopefully be more accurate.

    Args:
        dist (int): The distance to drive for.
    """
    db.reset()
    while True:
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        difference = left_reflection - right_reflection
        db.drive(FAST_SPEED, difference * 0.4)
        if db.distance() >= dist:
            break
    db.brake()


def drive_to_line(stretch: int, *, move_forward: bool) -> None:
    """Drive straight until it hits a line.

    Args:
        stretch (int): How far to drive before slowing down and enabling detection.
        move_forward (bool): Whether to go forward to get axle on the line
    """
    slow = False
    db.reset()
    while True:
        if not slow and db.distance() > stretch:
            slow = True
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        print(left_reflection, right_reflection)
        if left_reflection < BLACK_THRESHOLD and right_reflection < WHITE_THRESHOLD:
            if move_forward:
                db.straight(LIGHT_SENSOR_X, then=Stop.BRAKE)
            break
        if slow:
            db.drive(SLOW_SPEED, 0)
        else:
            db.drive(FAST_SPEED, 0)


def detect() -> list[bool]:
    """Scan the colors of the park elements.

    Returns:
        A list of 6 booleans. True means the park element is a lake element.
    """
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
            print(hsv)
            total_s += hsv.s
            total_v += hsv.v
        averages[i] = (total_s + total_v) / tally

    print(averages)
    sorted_averages = sorted([(value, index) for index, value in enumerate(averages)], reverse=True)
    park_elements = [False] * 6
    for a in sorted_averages[0:2]:
        park_elements[a[1]] = True
    return park_elements


def triple(park_elements: list[bool]) -> int | None:
    """Calculate where the first set of 3 park elements is.

    Args:
        park_elements (list[bool]): The park element slots. True means it is a lake element.
    """
    for i in range(len(park_elements) - 2):
        subset = park_elements[i : i + 3]
        if sum(subset) == 1:
            return i
    return None


def cage_up(*, blocking: bool) -> None:
    """Lift the cage up.

    Args:
        blocking (bool): Whether to block while moving the cage.
    """
    cage_motor.run_target(360, 145, wait=blocking)


def cage_down(*, blocking: bool) -> None:
    """Close the cage.

    Args:
        blocking (bool): Whether to block while moving the cage.
    """
    cage_motor.run_target(100, 53, wait=blocking)


def main() -> None:
    """The main function."""
    cage_down(blocking=True)
    db.curve(-170, -90)
    db.straight(-50)
    cage_up(blocking=True)

    # Collect starter kits
    db.curve(80, 69)
    db.curve(80, -69)
    simple_linetrack(273)
    cage_up(blocking=True)
    db.turn(90)
    # db.settings(straight_speed=SLOW_SPEED)
    db.straight(-170)
    cage_down(blocking=True)
    # db.settings(straight_speed=FAST_SPEED)
    db.straight(170)
    db.turn(90)
    linetrack("green", 25, 80, move_forward=False)
    db.curve(205, -90)

    # Detect park elements
    detected = detect()
    first_triple = triple(detected)
    print(detected, first_triple)
    db.curve(45, -90)
    linetrack("both", 25, 50, move_forward=False)

    # Release blocks
    cage_up(blocking=True)
    db.straight(-50)
    cage_down(blocking=True)
    db.curve(100, -60)
    db.straight(100)
    db.straight(-100)
    db.curve(-100, -60)
    db.curve(100, 60)
    db.straight(100)
    db.straight(-100)
    db.curve(-100, 60)
    db.straight(-100)
    db.turn(-90)

    # Collect first triple
    simple_linetrack((4 - first_triple) * (31.9 + 52.15) + 52.15)
    cage_up(blocking=False)
    db.turn(-90)
    db.straight(160)
    cage_down(blocking=True)
    db.straight(-160)
    db.turn(-90)
    linetrack("both", 100, 600, move_forward=True)
    db.turn(90)
    db.straight(160)
    cage_up(blocking=True)
    db.straight(-160)
    db.turn(90)


if __name__ == "__main__":
    main()
