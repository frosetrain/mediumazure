"""Medium Azure, RMJ24_292_01."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, hub_menu, wait
from ustruct import pack_into

BLACK_THRESHOLD = 25
WHITE_THRESHOLD = 60
LIGHT_SENSOR_X = 128
GRAB_FORWARD = 132

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


def linetrack(
    junction_type: str,
    start: int,
    stretch: int,
    *,
    move_forward: bool = True,
    linetrack: bool = True,
) -> None:
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
                db.straight(LIGHT_SENSOR_X + 2, then=Stop.BRAKE)
            else:
                db.brake()
            break
        difference = left_reflection - right_reflection if linetrack else 0
        if slow:
            db.drive(slow_speed, difference * 0.6)
        else:
            db.drive(fast_speed, difference * 0.4)


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
        db.drive(fast_speed, difference * 0.25)
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
            db.drive(slow_speed, 0)
        total = 0
        tally = 0
        while db.distance() < slot_end:
            db.drive(slow_speed, 0)
            tally += 1
            hsv = side_color_sensor.hsv()
            total += hsv.s + hsv.v
        averages[i] = total / tally

    print(averages)
    store_averages(averages)
    sorted_averages = sorted(enumerate(averages), key=lambda x: x[1], reverse=True)
    print(sorted_averages)
    park_elements = [1] * 6
    park_elements[sorted_averages[0][0]] = 2
    park_elements[sorted_averages[-1][0]] = 0
    print(park_elements)
    return park_elements


def calculate_grabs(park_elements: list[bool]) -> tuple[list[int], list[int]]:
    """Calculate where the grabs are.

    Args:
        park_elements (tuple[list[int], list[int]]): The park element slots. True means it is a lake element.

    Raises:
        ValueError: Something weird happened.
    """
    park_elements.append(park_elements[0])
    park_elements.append(park_elements[1])
    for i in range(6):
        subset = park_elements[i : i + 3]
        if sum(subset) == 4:
            if i == 4:  # wrapped around
                return ([5, -1], [(i - 2) % 6])
            elif i == 5:
                return ([6, 0], [(i - 2) % 6])
            elif i in (1, 2):
                return ([i + 1], [1, 4])
            else:
                return ([i + 1], [(i - 2) % 6])


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
    cage_motor.run_target(108, 150, wait=blocking)


def cage_down(*, blocking: bool) -> None:
    """Close the cage.

    Args:
        blocking (bool): Whether to block while moving the cage.
    """
    cage_motor.run_target(110, 52, wait=blocking)


def setup() -> None:
    """Read button presses to set the drivebase settings and breakpoint."""
    global fast_speed, slow_speed, starting_point
    hub.speaker.volume(50)
    # Set speed
    speed_levels = [200, 250, 300, 350, 420]
    selected_speed_level = hub_menu(*range(5))
    fast_speed = speed_levels[selected_speed_level]
    slow_speed = fast_speed / 2
    hub.speaker.play_notes(["C4/8"])
    # Set turn rate
    turnrate_levels = [120, 180, 240, 300, 360]
    selected_turnrate_level = hub_menu(*range(5))
    turn_rate = turnrate_levels[selected_turnrate_level]
    hub.speaker.play_notes(["D4/8"])
    db.settings(fast_speed, fast_speed * 4, turn_rate, turn_rate * 4)
    # Set starting point
    starting_point = hub_menu(*range(5))
    hub.speaker.play_notes(["E4/8"])
    wait(1000)


def main() -> None:
    """The main function."""
    print("Starting point", starting_point)
    # STARTING POINT 0
    if starting_point <= 0:
        # Collect starter kits
        db.curve(80, 69)
        db.curve(80, -69)
        simple_linetrack(267)
        cage_up(blocking=True)
        db.turn(90)
        db.straight(-132)
        cage_down(blocking=True)
        db.straight(132)
        db.turn(90)
        linetrack("green", 25, 80)
        db.curve(80, -90)
        db.straight(100)
        # Detect park elements
        detected = detect()
        first_grabs, second_grabs = calculate_grabs(detected)
        print(detected, first_grabs, second_grabs)
        db.curve(60, -90)
        linetrack("both", 10, 10, move_forward=False)

    # STARTING POINT 1
    if starting_point <= 1:
        # Release starter kits
        db.turn(180)
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
        db.turn(-145)
        db.straight(282)
        db.straight(-282)
        db.turn(-35)
        linetrack("right", 10, 10)

    # STARTING POINT 2
    if starting_point <= 2:
        # Collect first grabs
        simple_linetrack((5 - min(first_grabs)) * (31.9 + 52.15) + 55.95)
        cage_up(blocking=False)
        db.turn(-90)
        db.straight(GRAB_FORWARD)
        cage_down(blocking=True)
        db.straight(-GRAB_FORWARD)
        db.turn(-90)
        if len(first_grabs) == 2:
            simple_linetrack((max(first_grabs) - min(first_grabs)) * (31.9 + 52.15))
            cage_up(blocking=False)
            db.turn(90)
            db.straight(GRAB_FORWARD)
            cage_down(blocking=True)
            db.straight(-GRAB_FORWARD)
            db.turn(-90)

        # Head to South Park
        linetrack("both", 100, 400)
        db.turn(90)
        cage_up(blocking=False)
        db.straight(136)
        db.straight(-136)
        cage_down(blocking=False)
        db.turn(90)
        linetrack("right", 100, 580)

    # STARTING POINT 3
    if starting_point <= 3:
        # Collect second grabs
        simple_linetrack((5 - max(second_grabs)) * (31.9 + 52.15) + 55.95)
        cage_up(blocking=False)
        db.turn(-90)
        db.straight(GRAB_FORWARD)
        cage_down(blocking=True)
        db.straight(-GRAB_FORWARD)
        db.turn(90)
        if len(second_grabs) == 2:
            simple_linetrack((min(second_grabs) - max(second_grabs)) * (31.9 + 52.15))
            cage_up(blocking=False)
            db.turn(-90)
            db.straight(GRAB_FORWARD)
            cage_down(blocking=True)
            db.straight(-GRAB_FORWARD)
            db.turn(90)
        # Head to North Park
        linetrack("green", 30, 120, move_forward=False)
        db.curve(160, 90)
        linetrack("both", 50, 400, move_forward=False)
        db.straight(-30)
        db.turn(90)
        db.straight(853.2)
        db.turn(-90)
        cage_up(blocking=False)
        db.straight(160)
        db.straight(-164)
        db.turn(-90)
        db.straight(457)
        db.turn(90)
        db.straight(150)
        cage_down(blocking=True)
        db.straight(-150)
        db.turn(90)
        db.straight(457)
        db.turn(-90)
        cage_up(blocking=True)
        db.straight(160)
        db.straight(-160)
        db.turn(90)
        cage_down(blocking=False)
        # Head to e-bikes
        db.straight(242.675)
        db.curve(215.825, 90)
        db.straight(-420)


if __name__ == "__main__":
    # while True:
    # cage_up(blocking=True)
    # hub.speaker.beep()
    # wait(1000)
    # cage_down(blocking=True)
    # hub.speaker.beep()
    # wait(100)
    fast_speed = 300
    slow_speed = 150
    turn_rate = 240
    db.settings(fast_speed, fast_speed * 3, turn_rate, turn_rate * 4)
    starting_point = 0
    # setup()
    main()
