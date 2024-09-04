"""Medium Azure, RMJ24_292_01."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from ustruct import pack_into

FAST_SPEED = 250
SLOW_SPEED = 150
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


def linetrack(junction_type: str, start: int, stretch: int, *, move_forward: bool = True, linetrack: bool = True) -> None:
    """Line track.

    Args:
        junction_type (str): Left, both, right or green.
        start (str): How far to drive before going quickly.
        stretch (int): How far to drive before slowing down and enabling detection.
        move_forward (bool): Whether to go forward to get axle on the line. Defaults to True.
        linetrack (bool): Whether to linetrack. Defaults to True.
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
        if hit and db.distance() >= stretch:
            if move_forward:
                db.straight(LIGHT_SENSOR_X + 6, then=Stop.BRAKE)
            else:
                db.brake()
            break
        difference = left_reflection - right_reflection if linetrack else 0
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
        db.drive(FAST_SPEED, difference * 0.25)
        if db.distance() >= dist:
            break
    db.brake()


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
            db.drive(SLOW_SPEED, 0)
        total = 0
        tally = 0
        while db.distance() < slot_end:
            db.drive(SLOW_SPEED, 0)
            tally += 1
            hsv = side_color_sensor.hsv()
            total += hsv.s + hsv.v
        averages[i] = total / tally

    print(averages)
    store_averages(averages)
    sorted_averages = sorted(enumerate(averages), key=lambda x: x[1], reverse=True)
    park_elements = [False] * 6
    for a in sorted_averages[0:2]:
        park_elements[a[0]] = True
    return park_elements


def triple(park_elements: list[bool]) -> int:
    """Calculate where the first set of 3 park elements is.

    Args:
        park_elements (list[bool]): The park element slots. True means it is a lake element.

    Raises:
        ValueError: No triplets were found.
    """
    for i in range(4):
        subset = park_elements[i : i + 3]
        if sum(subset) == 1:
            if i == 0:
                return 3
            else:
                return i


def store_averages(averages: list[int]) -> None:
    """Store the average s+v values in the hub's persistent storage.

    Args:
        averages (list[int]): An array of average values.
    """
    buf = bytearray(6 * 2)  # 2 bytes for each unsigned short
    for i, val in enumerate(averages):
        pack_into("H", buf, i * 2, int(val))
    hub.system.storage(0, write=buf)


def cage_up(*, blocking: bool) -> None:
    """Lift the cage up.

    Args:
        blocking (bool): Whether to block while moving the cage.
    """
    cage_motor.run_time(500, 0.3)
    cage_motor.run_target(96, 142, wait=blocking)


def cage_down(*, blocking: bool) -> None:
    """Close the cage.

    Args:
        blocking (bool): Whether to block while moving the cage.
    """
    cage_motor.run_target(96, 52, wait=blocking)


def main() -> None:
    """The main function."""
    # Collect starter kits
    db.curve(80, 69)
    db.curve(80, -69)
    simple_linetrack(271)
    cage_up(blocking=True)
    db.turn(90)
    # db.settings(straight_speed=SLOW_SPEED)
    db.straight(-132)
    cage_down(blocking=True)
    # db.settings(straight_speed=FAST_SPEED)
    db.straight(132)
    db.turn(90)
    linetrack("green", 25, 80)
    db.curve(80, -90)
    db.straight(100)

    # Detect park elements
    detected = detect()
    first_triple = triple(detected)
    print(detected, first_triple)
    db.curve(60, -90)
    linetrack("both", 10, 10, move_forward=False)
    db.turn(180)

    # Release blocks
    cage_up(blocking=True)
    db.straight(-55)
    db.straight(108)
    # First apartment
    db.turn(130)
    db.straight(260)
    db.straight(-260)
    db.turn(100)
    # Second apartment
    db.straight(260)
    db.straight(-260)
    db.turn(-140)
    # House
    db.straight(100)
    db.turn(90)
    db.straight(250)
    db.straight(-100)
    cage_down(blocking=False)
    db.straight(-144)
    db.turn(90)
    simple_linetrack(350)
    db.turn(-150)
    db.straight(275)
    db.straight(-275)
    db.turn(-30)
    linetrack("right", 10, 10)
    # wait(5000)

    # Collect first triple
    simple_linetrack((4 - first_triple) * (31.9 + 52.15) + 52.15)
    cage_up(blocking=False)
    # wait(2000)
    db.turn(-90)
    db.straight(136)
    cage_down(blocking=True)
    db.straight(-136)
    db.turn(-90)
    # Head to South Park
    linetrack("both", 100, 600)
    db.turn(90)
    db.straight(136)
    cage_up(blocking=True)
    db.straight(-136)
    cage_down(blocking=False)
    db.turn(90)

    # Collect second triple
    linetrack("right", 100, 580)
    if first_triple == 3:
        simple_linetrack(4 * (31.9 + 52.15) + 52.15)
        cage_up(blocking=False)
        db.turn(-90)
        db.straight(136)
        cage_down(blocking=True)
        db.straight(-136)
        db.turn(90)
    else:
        simple_linetrack(1 * (31.9 + 52.15) + 52.15)
        cage_up(blocking=False)
        db.turn(-90)
        db.straight(136)
        cage_down(blocking=True)
        db.straight(-136)
        db.turn(90)
        simple_linetrack(3 * (31.9 + 52.15))
        db.turn(-90)
        cage_up(blocking=True)
        db.straight(136)
        cage_down(blocking=True)
        db.straight(-136)
        db.turn(90)

    # Head to North Park
    linetrack("green", 30, 120, move_forward=False)
    db.curve(160, 90)
    linetrack("both", 50, 400, move_forward=False)
    db.straight(-30)
    db.turn(90)
    db.straight(853.2)
    db.turn(-90)
    db.straight(160)
    cage_up(blocking=True)
    db.straight(-160)
    cage_down(blocking=False)
    # Head to e-bikes
    db.turn(90)
    db.straight(242.675)
    db.curve(215.825, 90)
    db.straight(-420)


if __name__ == "__main__":
    main()
