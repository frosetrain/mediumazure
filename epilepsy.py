from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
left_color_sensor = ColorSensor(Port.D)
right_color_sensor = ColorSensor(Port.C)
ultrasonic = UltrasonicSensor(Port.E)
helikopter = Motor(Port.F)
db = DriveBase(left_motor, right_motor, 56, 16 * 8)
db.settings(
    straight_speed=400,
    straight_acceleration=4000,
    turn_rate=360,
    turn_acceleration=3600,
)

matrix0 = [
    [0, 100, 0, 100, 0],
    [100, 0, 100, 0, 100],
    [0, 100, 0, 100, 0],
    [100, 0, 100, 0, 100],
    [0, 100, 0, 100, 0],
]
matrix1 = [
    [100, 0, 100, 0, 100],
    [0, 100, 0, 100, 0],
    [100, 0, 100, 0, 100],
    [0, 100, 0, 100, 0],
    [100, 0, 100, 0, 100],
]
step = 0


def epilepsy(cointensity):
    global step
    ultrasonic_lights = [0, 0, 0, 0]
    ultrasonic_lights[step] = 100
    left_color_sensor.lights.on(100)
    right_color_sensor.lights.on(100)
    hub.display.icon(matrix0)
    ultrasonic.lights.on(ultrasonic_lights)
    wait(cointensity)
    left_color_sensor.lights.off()
    right_color_sensor.lights.off()
    hub.display.icon(matrix1)
    wait(cointensity)
    step += 1
    step %= 4


def main():
    helikopter.dc(100)
    db.straight(150, wait=False)
    while not db.done():
        epilepsy(50)
    db.straight(-150, wait=False)
    while not db.done():
        epilepsy(50)
    db.curve(-100, 90, wait=False)
    while not db.done():
        epilepsy(50)
    db.curve(100, -90, wait=False)
    while not db.done():
        epilepsy(50)
    db.turn(180, wait=False)
    while not db.done():
        epilepsy(50)
    db.curve(100, -90, wait=False)
    while not db.done():
        epilepsy(50)
    db.curve(-100, 90, wait=False)
    while not db.done():
        epilepsy(50)
    db.turn(720, wait=False)
    while not db.done():
        epilepsy(50)
    ultrasonic.lights.off()
    helikopter.stop()
    wait(1000)
    helikopter.run_angle(720, 360)
    helikopter.run_angle(720, -360)
    db.drive(100, 0)
    for i in range(4):
        hub.display.icon(matrix0)
        helikopter.run_angle(720, -360)
        hub.display.off()
        helikopter.run_angle(720, 360)
    db.stop()
    db.curve(100, 180)
    db.drive(100, 0)
    hub.display.text("Medium Azure")
    db.stop()
    helikopter.dc(100)
    db.turn(-720, wait=False)
    while not db.done():
        epilepsy(0)
    helikopter.stop()


if __name__ == "__main__":
    main()
