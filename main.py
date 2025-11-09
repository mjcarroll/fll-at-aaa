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
WHEEL_DIAM = 624 # millimeters*10
WHEEL_CIRC = int(WHEEL_DIAM * math.pi)
print("WHEEL_CIRC: ", WHEEL_CIRC)
MAX_TURN_VELOCITY = 300
MIN_TURN_VELOCITY = 50

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
    """
    M12: 30 (20+10)
    Flag: 10
    LINE UP [RED]: 2 squares from right on inside line (right corner robot)
    """
    flag_turn = -350
    await drive_straight(500, velocity=450, acceleration=500)
    await motor.run_for_degrees(ACC_HIGH, flag_turn, 500)
    await runloop.sleep_ms(100)
    await drive_straight(-100)
    await motor.run_for_degrees(ACC_LOW, -240, 500)
    await drive_straight(-150, velocity=450)
    motor.run_for_degrees(ACC_LOW, 240, 500)
    motor.run_for_degrees(ACC_HIGH, -flag_turn, 500)
    await drive_straight(-300, velocity=450)



async def surface_brushing_map_reveal():
    """
    M02: 30 (10 pt/obj)
    M01: 30 (10 pt/obj + 10)


    LINE UP [RED]: 11 squares from left (left corner robot)
    """
    await drive_straight(730, velocity=300, acceleration=500)
    await turn_to_angle(-342)
    await runloop.sleep_ms(10)
    await drive_straight(140, velocity=200)
    await motor.run_for_degrees(ACC_HIGH, 300, 500) # M02 (Map Reveal)
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

async def cross_field():
    motor.run_for_degrees(ACC_HIGH, -180, 250)
    await drive_straight(870)
    await turn_to_angle(900)
    await motor.run_for_degrees(ACC_HIGH, 180, 250)
    await drive_straight(160)
    # We are now under the minecart
    await motor.run_for_degrees(ACC_HIGH, -360, 250)
    await motor.run_for_degrees(ACC_HIGH, 180, 250)
    motor.run_for_degrees(ACC_HIGH, 180, 250)
    await turn_to_angle(470)
    await drive_straight(180)

    # Lifting the statue
    await motor.run_for_degrees(ACC_HIGH, -100, 100)
    await turn_to_angle(-100)
    await motor.run_for_degrees(ACC_HIGH, -300, 500)
    await drive_straight(-100)
    await turn_to_angle(-460)
    await drive_straight(800)
    await turn_to_angle(540)

    #await motor.run_for_degrees(ACC_HIGH, 400, 1000)
    #await motor.run_for_degrees(ACC_HIGH, -80, 250)
    await drive_straight(500)





async def cross_field2():
    """
    M03: 30
    M04: 10 (passive)
    M13: 30
    M09: 20 (roof)


    LINE UP [RED]: 9 squares from left (left corner robot)
    """
    await drive_straight(710,velocity=600)
    await turn_to_angle(620)
    await drive_straight(185)
    await motor.run_for_degrees(ACC_HIGH, -360, 250) # raise arm to move M03 (Mineshaft Explorer)
    #await runloop.sleep_ms(200)
    await motor.run_for_degrees(ACC_HIGH, 180, 250) # lower arm
    await turn_to_angle(280)
    await drive_straight(50)
    await turn_to_angle(400)
    await motor.run_for_degrees(ACC_HIGH, 180, 250)
    await drive_straight(80)

    # setup for M13 (Statue)
    if False:
        await drive_straight(20)
        await turn_to_angle(700) 
        await drive_straight(105)
        await turn_to_angle(60)
        await drive_straight(25)
    #await turn_to_angle(-90)
    await motor.run_for_degrees(ACC_HIGH, -230, 300) # lift statue
    await runloop.sleep_ms(500)
    #await turn_to_angle(-40)
    await motor.run_for_degrees(ACC_HIGH, -150, 300) # continue to lift statue
    await runloop.sleep_ms(200)
    await motor.run_for_degrees(ACC_HIGH, -150, 300) # ensure arm raised enough to not obstruct

    await drive_straight(-150)
    await turn_to_angle(-450)
    await drive_straight(600)
    return

    # setup to cross field
    await turn_to_angle(-10)
    await turn_to_angle(30)
    await drive_straight(-60)
    await turn_to_angle(-650)
    await drive_straight(540,velocity=600)
    await turn_to_angle(430)
    await drive_straight(290)
    await turn_to_angle(90)
    await drive_straight(290,velocity=100) # bump M09 (What's On Sale?)
    await drive_straight(-130)
    await turn_to_angle(-175)
    await drive_straight(800,velocity=1000) # drive to blue home area
    return

async def who_lived_and_forge():
    """
    M06: 30 (10 pt/rock)
    M05: 30
    M07: 30


    LINE UP [BLUE]: 2 squares from left (left corner robot)
    """
    await drive_straight(705, velocity=600)
    await turn_to_angle(450)
    await drive_straight(30)
    color_check = True
    while(color_check):
        await drive_straight(5)
        if color_sensor.color(SNS_LEFT) is color.BLACK and color_sensor.color(SNS_RIGHT) is color.BLACK:
            color_check = False
    await drive_straight(-30)
    await turn_to_angle(-700, sleep_ms=200) # dump rocks M06 (Forge)
    await drive_straight(30)
    await turn_to_angle(-140) # flip M05 (Who Lived Here?)
    await drive_straight(-50)
    await turn_to_angle(-510) # move rocks into home area
    await drive_straight(-420,velocity= 750)
    await turn_to_angle(640) # set up M07 (Heavy Lifting)
    await motor.run_for_degrees(ACC_LOW, -135, 500) # drop armNEED TO DOUBLE CHECK
    await drive_straight(35)
    await turn_to_angle(-100)
    await motor.run_for_degrees(ACC_LOW, 135, 100) # pick up millstone
    await turn_to_angle(100)
    await drive_straight(-520, velocity= 750) # return home
    return

async def tip_the_scales():
    """
    M10: 30


    LINE UP [BLUE]: 9 squares from left on launch/home border (left corner robot)
    """
    await drive_straight(700, velocity= 1000)
    await turn_to_angle(-900)
    await drive_straight(-150) # tip scale
    await drive_straight(150) # remove pan
    await turn_to_angle(-700)
    await drive_straight(650, velocity=1000) # return home
    return

async def forum():
    """
    M14: 25 (5 pt/obj)


    LINE UP [BLUE]: 3 black lines from left (left corner robot), 2 black lines on arc (right cage wall)
    """
    await drive_straight(-1075, velocity= 800)
    await drive_straight(200, velocity= 1500) # backup to not touch artifacts
    return

##### RUN LIST
# keeps a list of run name and run function
"""
Point totals:    405
Inspection:20
Precision:    50
Run 1:        60
Run 2:        40
Run 3:        90
Run 4:        90
Run 5:        30
Run 6:        25
"""
runs = [
    ("2", boat),


    #("0", artbots),
    # From the RED SIDE
    ("1", surface_brushing_map_reveal),
    ("2", boat),
    ("3", cross_field),
    # From the BLUE SIDE
    ("4", who_lived_and_forge),
    ("5", tip_the_scales),
    ("6", forum),
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
