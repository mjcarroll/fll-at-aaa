######### IMPORTS
import math
import runloop
import motor
import motor_pair

from hub import motion_sensor
from hub import port

from app import linegraph
import color

######### CONSTANTS
PAIR_IDX = 0
LEFT_MOTOR = port.A
RIGHT_MOTOR = port.E
ACC_LOW = port.C
ACC_HIGH = port.F
SNS_LEFT = port.D
SNS_RIGHT = port.B
WHEEL_DIAM = 624# millimeters*10
#WHEEL_DIAM = 550
WHEEL_CIRC = int(WHEEL_DIAM * math.pi)
print("WHEEL_CIRC: ", WHEEL_CIRC)
MAX_TURN_VELOCITY = 300
MIN_TURN_VELOCITY = 50

######### HARDWARE SETUP
motor_pair.pair(PAIR_IDX, LEFT_MOTOR, RIGHT_MOTOR)

def sum_wheels(curTime) -> int:
    L = abs(motor.relative_position(LEFT_MOTOR))
    R = abs(motor.relative_position(RIGHT_MOTOR))
    #linegraph.plot(color.YELLOW, curTime, L)
    #linegraph.plot(color.TURQUOISE, curTime, R)
    return int((L + R) / 2)

async def drive_straight(
    target_distance: int,
    velocity: int = 500,
    dt: int = 10,
):
    degrees_to_move = int((3600 * target_distance) / WHEEL_CIRC)
    print("drive straight: ", target_distance, degrees_to_move)
    motor.reset_relative_position(LEFT_MOTOR, 0); motor.reset_relative_position(RIGHT_MOTOR, 0)

    error = 0
    integrator = 0
    windup = 100
    kp = 0.1
    ki = 5

    linegraph.clear_all()
    curTime = 0
    sumPos = sum_wheels(curTime)
    motion_sensor.reset_yaw(0)

    # Heading is more positive on left turn
    # Heading is more negative on right turn

    while sumPos < degrees_to_move:
        curTime = curTime + dt
        curHeading = motion_sensor.tilt_angles()[0]

        error = -curHeading
        integrator = integrator + (error * dt)/1000

        if (integrator > windup): integrator = windup
        if (integrator < -windup): integrator = -windup

        correction = kp * error + ki * integrator

        #linegraph.plot(color.BLACK, curTime, curHeading)
        linegraph.plot(color.BLUE, curTime, error)
        linegraph.plot(color.GREEN, curTime, integrator)

        linegraph.plot(color.RED, curTime, kp*error)
        linegraph.plot(color.MAGENTA, curTime, ki*integrator)

        motor_pair.move_tank(PAIR_IDX,
            velocity - int(velocity * correction / 100),
            velocity + int(velocity * correction / 100)
        )
        await runloop.sleep_ms(dt)
        sumPos = sum_wheels(curTime)
    motor_pair.stop(PAIR_IDX)

async def drive_straight2(
    target_distance: int,
    sleep_ms: int = 10,
    velocity: int = 500,
    acceleration: int = 1000,
    curve: int = 0,
):
    """
    target_distance: Distance to move in millimeters
    sleep_ms: time to sleep at end
    velocity: speed to move at
    acceleration: how much to accelerate
    """
    degrees_to_move = int((3600 * target_distance) / WHEEL_CIRC)
    print("drive straight: ", target_distance, degrees_to_move)
    await motor_pair.move_for_degrees(
        PAIR_IDX,
        degrees_to_move,
        curve,
        velocity=velocity,
        acceleration=acceleration,
        stop=motor.SMART_BRAKE,
    )
    if sleep_ms > 0:
        await runloop.sleep_ms(sleep_ms)

async def main():
    linegraph.clear_all()
    #await drive_straight(1802, velocity=500, dt=1)
    await drive_straight2(1802, velocity=500)
    pass


runloop.run(main())
