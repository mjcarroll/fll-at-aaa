from hub import port
import runloop
import motor_pair

from hub import motion_sensor
from time import sleep

PAIR_IDX = 0
LEFT_MOTOR = port.E
RIGHT_MOTOR = port.F
WHEEL_DIAM = 82         # millimeters

motor_pair.pair(PAIR_IDX, LEFT_MOTOR, RIGHT_MOTOR)

def signum(value: int) -> int:
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0

async def turn_to_angle(target_yaw: int, sleep_ms: int = 10):
    """
    turn_angle: Angle to turn to in decidegrees
                (+) is to the right
                (-) is to the left
                abs(turn_angle) < 1800
    """
    motion_sensor.reset_yaw(0)
    error = target_yaw
    print("turn to angle: ", target_yaw)

    while True: 
        current_yaw = motion_sensor.tilt_angles()[0]
        error = target_yaw - current_yaw
        if abs(error) < 5:
            break
        steering = -100 * signum(error)
        motor_pair.move(PAIR_IDX, steering)
    motor_pair.stop(PAIR_IDX)
    await runloop.sleep_ms(sleep_ms)

async def drive_straight(target_distance: int, sleep_ms: int = 10):
    """
    target_distance: Distance to move in millimeters
    """
    degrees_to_move = int((3600 * target_distance) / 2576)
    print("drive straight: ", target_distance, degrees_to_move)
    await motor_pair.move_for_degrees(PAIR_IDX, degrees_to_move, 0)
    await runloop.sleep_ms(sleep_ms)


async def main():
    await drive_straight(95)
    await turn_to_angle(400)
    await drive_straight(500)
    await drive_straight(-500)
    await turn_to_angle(-400)

runloop.run(main())