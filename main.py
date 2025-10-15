######### IMPORTS
import math
import color_sensor
import runloop
import motor
import motor_pair

import color

from hub import button
from hub import light
from hub import light_matrix
from hub import motion_sensor
from hub import port

######### CONSTANTS
PAIR_IDX = 0
LEFT_MOTOR = port.A
RIGHT_MOTOR = port.E
ACC_LOW = port.C
ACC_HIGH = port.F
SNS_LEFT = port.D
SNS_RIGHT = port.B
WHEEL_DIAM = 624# millimeters*10
WHEEL_CIRC = int(624 * math.pi)
print("WHEEL_CIRC: ", WHEEL_CIRC)
MAX_TURN_VELOCITY = 300
MIN_TURN_VELOCITY = 50
DO_SOIL = False

######### HARDWARE SETUP
motor_pair.pair(PAIR_IDX, LEFT_MOTOR, RIGHT_MOTOR)

######### COMMON FUNCTIONS
async def turn_to_angle(target_yaw: int, sleep_ms: int = 10):
    """
    turn_angle: Angle to turn to in degrees*10
                (+) is to the right
                (-) is to the left
                angle must be smaller than 180
    """
    target_yaw = -target_yaw
    motion_sensor.reset_yaw(0)
    error = target_yaw
    print("turn to angle: ", target_yaw)

    while True:
        current_yaw = motion_sensor.tilt_angles()[0]
        error = current_yaw - target_yaw

        # if we are within half a degree, then stop
        if abs(error) < 5:
            break
        # steering goes between -100 and 100
        # decides how big the curve is
        # 100 means turn in place
        steering = 0
        if error < 0:
            steering = -100
        elif error > 0:
            steering = 100

        v = abs(int(error / 2))
        if v > MAX_TURN_VELOCITY:
            v = MAX_TURN_VELOCITY
        if v > 0 and v < MIN_TURN_VELOCITY:
            v = MIN_TURN_VELOCITY
        #print(error, target_yaw, current_yaw, v, steering)
        motor_pair.move(PAIR_IDX, steering, velocity=v)
    motor_pair.stop(PAIR_IDX)
    await runloop.sleep_ms(sleep_ms)


async def drive_straight(
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

####### RUNS
async def artbots():
    await light_matrix.write("ARTBOTS")

async def boat():
    await drive_straight(650, velocity=450)
    await drive_straight(-650, velocity=450)

async def surface_brushing_map_reveal():
    await drive_straight(754, velocity=600)
    await turn_to_angle(-345)
    await runloop.sleep_ms(10)
    await drive_straight(145, velocity=200)
    await motor.run_for_degrees(ACC_HIGH, 300, 500)
    await drive_straight(-30, acceleration=1500)
    await drive_straight(-135)
    await turn_to_angle(-430)
    await drive_straight(120)
    await motor.run_for_degrees(ACC_LOW, 750, 500)
    await motor.run_for_degrees(ACC_LOW, -750, 500)
    await drive_straight(-100)
    await motor.run_for_degrees(ACC_HIGH, -550, 500)
    await drive_straight(40)
    await motor.run_for_degrees(ACC_HIGH, 500, 500)
    await drive_straight(-200)
    await turn_to_angle(-1300)
    await motor.run_for_degrees(ACC_HIGH, -400, 500)

async def who_lived_and_forge():
    await drive_straight(790)
    await turn_to_angle(470)
    await drive_straight(50)
    color_check = True
    while(color_check):
        await drive_straight(5)
        if color_sensor.color(SNS_LEFT) is color.WHITE and color_sensor.color(SNS_RIGHT) is color.WHITE:
            color_check = False
    await turn_to_angle(-500, sleep_ms=200)
    await drive_straight(50)
    await turn_to_angle(-375)
    await drive_straight(-100)
    await turn_to_angle(-550)
    await drive_straight(-420)
    await turn_to_angle(900)
    await drive_straight(40)
    await turn_to_angle(-200)
    await drive_straight(-20)
    return

async def tip_the_scales():
    await drive_straight(775)
    await turn_to_angle(-900)
    await drive_straight(-200)
    await drive_straight(200)
    await turn_to_angle(-700)
    await drive_straight(670, velocity=1000)
    await turn_to_angle(125)
    await runloop.sleep_ms(2000)
    await drive_straight(-1250, velocity=1000)
    await drive_straight(100, acceleration=1500)
    pass

##### RUN LIST
# keeps a list of run name and run function
runs = [
    #("0", artbots),
    # From the RED SIDE
    ("1", surface_brushing_map_reveal),
    ("2", boat),
    # From the BLUE SIDE
    ("3", who_lived_and_forge),
    ("4", tip_the_scales)
]

###### MAIN FUNCTION
async def main():
    light.color(light.POWER, color.PURPLE)
    changed = True
    running = False
    run_idx = 0

    # Main Program - Run forever
    # If left button is pressed, go to next program
    # If right button is pressed, run current program
    # After running, reset screen
    while True:
        if button.pressed(button.LEFT):
            # wait for button to let go (from guide)
            while button.pressed(button.LEFT):
                pass
            # when left button is pressed, advance
            # program and tell screen to change
            changed = True
            run_idx = run_idx + 1
            if run_idx >= len(runs):
                run_idx = 0

        if button.pressed(button.RIGHT):
            # wait for button to let go (from guide)
            while button.pressed(button.RIGHT):
                pass
            # when right button is pressed, mark running
            running = True

        run_name, run_function = runs[run_idx]
        if running:
            light.color(light.POWER, color.GREEN)
            print("Run: ", run_name)
            await run_function()
            print("Run: ", run_name, " done!")
            running = False
            changed = True
        else:
            light.color(light.POWER, color.PURPLE)

        if changed:
            # without this, the screen flashes
            light_matrix.write(run_name)
            changed = False

runloop.run(main())
