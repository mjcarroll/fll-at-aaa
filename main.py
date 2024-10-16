from hub import port
import runloop
import motor
import motor_pair

from hub import motion_sensor
from time import sleep

PAIR_IDX = 0
LEFT_MOTOR = port.A
RIGHT_MOTOR = port.E
ACC_LOW = port.C
ACC_HIGH = port.F
SNS_LEFT = port.D
SNS_RIGHT= port.B
# WHEEL_DIAM = 624        # millimeters
MAX_VELOCITY = 300
MIN_VELOCITY = 50

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
    target_yaw = -target_yaw
    motion_sensor.reset_yaw(0)
    error = target_yaw
    print("turn to angle: ", target_yaw)

    while True:
        current_yaw = motion_sensor.tilt_angles()[0]
        error = current_yaw - target_yaw
        if abs(error) < 5:
            break
        steering = 100 * signum(error)
        v = abs(int(error/2))

        if v > MAX_VELOCITY:
            v = MAX_VELOCITY
        if v > 0 and v < MIN_VELOCITY:
            v = MIN_VELOCITY
        print (error, target_yaw, current_yaw, v, steering)

        motor_pair.move(PAIR_IDX, steering, velocity=v)
    motor_pair.stop(PAIR_IDX)
    await runloop.sleep_ms(sleep_ms)

async def drive_straight(target_distance: int, sleep_ms: int = 10, velocity=500, acceleration=1000):
    """
    target_distance: Distance to move in millimeters
    """
    degrees_to_move = int((3600 * target_distance) / 1958)
    print("drive straight: ", target_distance, degrees_to_move)
    await motor_pair.move_for_degrees(PAIR_IDX, degrees_to_move, 0, velocity=velocity, acceleration=1000, stop=motor.SMART_BRAKE)
    await runloop.sleep_ms(sleep_ms)


async def main():
    await drive_straight(635)
    await turn_to_angle(900)
    await drive_straight(100)
    await motor.run_for_degrees(ACC_LOW, -135, 600)
    await drive_straight(-110)
    await turn_to_angle(-750)
    await drive_straight(-150)
    await turn_to_angle(-500) #need timeout
    await turn_to_angle(-200)
    await turn_to_angle(-900)
    await drive_straight(-400)

async def run1():
    """
    Drive out, pull the soil sample, get the squid, return home
    """
    await drive_straight(755)
    await turn_to_angle(900)
    await drive_straight(79)
    
    await motor.run_for_degrees(ACC_LOW, -200, 600)
    await drive_straight(-110)
    await turn_to_angle(-900)
    await drive_straight(-490)
    await turn_to_angle(-430)
    await drive_straight(240)
    await drive_straight(-570)


async def run2():
    await drive_straight(440)
    await turn_to_angle(380)
    await drive_straight(32)
    motor_pair.move(PAIR_IDX, 100, velocity=75)
    await motor.run_for_time(ACC_LOW, 1500, 175)
    motor_pair.stop(PAIR_IDX)

async def run3():
    await drive_straight(-370)
    await drive_straight(300)
    await turn_to_angle(200)
    await drive_straight(-220, velocity=600, acceleration=5500)
    await turn_to_angle(350)
    await drive_straight(-230, velocity=625, acceleration=5500)
    await turn_to_angle(-500)
    await drive_straight(-190, velocity=650, acceleration=5500)
    await drive_straight(70)
    await turn_to_angle(900)
    await drive_straight(-150, velocity=675, acceleration=6000)
    await drive_straight(50)
    await turn_to_angle(350)
    await drive_straight(-150)
    await drive_straight(50)
    await drive_straight(-50)
    await drive_straight(150)
    await turn_to_angle(-1300)
    await drive_straight(70)
    motor.run(ACC_LOW, 60)
    await drive_straight(25)


runloop.run(run3())
