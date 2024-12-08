######### IMPORTS
import math

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
        print(error, target_yaw, current_yaw, v, steering)
        motor_pair.move(PAIR_IDX, steering, velocity=v)
    motor_pair.stop(PAIR_IDX)
    await runloop.sleep_ms(sleep_ms)


async def drive_straight(
    target_distance: int,
    sleep_ms: int = 10,
    velocity: int = 500,
    acceleration: int = 1000,
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
        0,
        velocity=velocity,
        acceleration=acceleration,
        stop=motor.SMART_BRAKE,
    )
    await runloop.sleep_ms(sleep_ms)


async def reset():
    """
    Reset the arm motors to a known position
    """
    low_set = False
    high_set = False
    while not low_set and not high_set:
        # Run both motors into the ground
        v_low = motor.run_for_degrees(ACC_LOW, -360, 1000)
        v_high = await motor.run_for_degrees(ACC_HIGH, -360, 1000)
        v_low = await v_low

        if v_low[0] == motor.STALLED:
            print("ACC_LOW STALLED")
            low_set = True
        if v_high[0] == motor.STALLED:
            print("ACC_HIGH STALLED")
            high_set = True
    # return to starting position
    motor.run_for_degrees(ACC_HIGH, 300, 400)
    await motor.run_for_degrees(ACC_LOW, 310, 400)


async def motor_test_up():
    """
    Move both arm motors 30 degrees
    """
    motor.run_for_degrees(ACC_HIGH, 30, 500)
    await motor.run_for_degrees(ACC_LOW, 30, 500)


async def motor_test_down():
    """
    Move both arm motors 30 degrees
    """
    motor.run_for_degrees(ACC_HIGH, -30, 500)
    await motor.run_for_degrees(ACC_LOW, -30, 500)

####### RUNS
async def artbots():
    await light_matrix.write("ARTBOTS")

async def red_pickup():
    """
    * Get the squid
    * Get the krill
    * Get the coral
    * Get the soil sample
    * Flip the boat
    """
    vel = 600
    accel = 5500

    # drive straight out and push the squid
    await drive_straight(-380, velocity=vel)

    # back up
    await drive_straight(100)
    await turn_to_angle(620)
    await drive_straight(-450)

    await turn_to_angle(651)

    await drive_straight(-70)
    await drive_straight(150)
    await turn_to_angle(-1250)
    await drive_straight(100)
    await motor.run_for_degrees(ACC_LOW, 60, 500)
    await drive_straight(90)

    # Go home!
    await drive_straight(-140)
    await motor.run_for_degrees(ACC_LOW, 150, 500)

    await turn_to_angle(350)
    motor.run_for_degrees(ACC_LOW, -60, 500)
    await drive_straight(750)


    return

    # turn and get first krill
    await turn_to_angle(200)
    await drive_straight(-220, velocity=vel, acceleration=accel)

    # turn and get coral
    await turn_to_angle(350)
    await drive_straight(-230, velocity=vel, acceleration=accel)

    # turn and get second krill (near sonar)
    await turn_to_angle(-500)
    await drive_straight(-190, velocity=vel, acceleration=accel)
    await drive_straight(70)

    # Turn and get last krill
    await turn_to_angle(850)
    await drive_straight(-150, velocity=vel, acceleration=accel)
    await drive_straight(50)

    # Turn and get sample
    await turn_to_angle(350)
    await drive_straight(-150)
    await drive_straight(50)
    # await drive_straight(-50)

    # Back up
    await drive_straight(110)
    await turn_to_angle(-1270)
    await drive_straight(70)
    await motor.run_for_degrees(ACC_LOW, 60, 500)
    await drive_straight(90)

    # Go home!
    await drive_straight(-60)
    await turn_to_angle(400)
    motor.run_for_degrees(ACC_LOW, -60, 500)
    await drive_straight(600, velocity=vel, acceleration=accel)


async def send_sub_and_lanternfish():
    """
    originally we used two arms to lift, but got it working with one
    uncomment to use other arm
    """
    motor.run_for_degrees(ACC_HIGH, -175, 500)
    # motor.run_for_degrees(ACC_LOW, -200, 500)
    await drive_straight(865)
    for ii in range(0, 10):
        motor.run_for_degrees(ACC_HIGH, 50, 1000)
        # motor.run_for_degrees(ACC_LOW, 25, 1000)
        await runloop.sleep_ms(200)
    await drive_straight(-260)
    await turn_to_angle(-450)
    await motor.run_for_degrees(ACC_LOW, -300, 400)
    await drive_straight(495)
    await motor.run_for_degrees(ACC_LOW, 300, 400)

    if DO_SOIL:
        # this sometimes works, decide if we want to keep it or not
        # set DO_SOIL at top
        await drive_straight(-150)
        await motor.run_for_degrees(ACC_HIGH, -360, 1000)
        await motor.run_for_degrees(ACC_HIGH, 160, 400)
        await drive_straight(300)
        await motor.run_for_degrees(ACC_HIGH, 160, 400)
        await drive_straight(-500)
        await turn_to_angle(-300)
        await drive_straight(-500)
    else:
        # just go home
        await drive_straight(-50)
        await turn_to_angle(550)
        await drive_straight(-500, velocity=600)
        await turn_to_angle(-300)
        await drive_straight(-500, velocity=600)
        await motor.run_for_degrees(ACC_LOW, -200, 400)
        await motor.run_for_degrees(ACC_HIGH, 200, 400)


async def sonar_and_critter():
    # Drive to sonar and put arm down
    await drive_straight(760)
    await motor.run_for_degrees(ACC_LOW, -185, 650)
    # pull the first sonar
    await drive_straight(-100)
    # Turn to move arm
    await turn_to_angle(-190)
    # Push second sonar
    await drive_straight(140)
    await motor.run_for_degrees(ACC_LOW, 185, 650)

    # Drive to critter drop off
    await drive_straight(-70)
    await turn_to_angle(-460)
    await drive_straight(220)
    await turn_to_angle(-300)

    # Drop off critter
    await motor.run_for_degrees(ACC_HIGH, 600, 200)
    await motor.run_for_degrees(ACC_HIGH, -600, 650)

    # go home
    await turn_to_angle(450)
    await drive_straight(-200)
    await turn_to_angle(500)
    await drive_straight(-650, velocity=600)


# 4 + 5 + 6 from dylan
async def run4():
    """
    Drive out and move the crab trap
    """
    await drive_straight(250)
    await turn_to_angle(400)
    await drive_straight(200)
    await turn_to_angle(-200)
    await drive_straight(200)
    await turn_to_angle(-200)
    await drive_straight(-500, velocity=1000)
    await turn_to_angle(450)
    await drive_straight(-300, velocity=1000)


async def run5():
    """
    Flip up the crab trap
    """
    await drive_straight(400)
    motor.run_for_degrees(ACC_LOW, 200, 1000)
    await drive_straight(-500)


async def run6():
    """
    Push the crab trap over
    """
    await drive_straight(650, velocity=1000, acceleration=500)
    await motor.run_for_degrees(ACC_LOW, 100, 1000)
    await drive_straight(-150)
    await motor.run_for_degrees(ACC_LOW, -100, 1000)
    await turn_to_angle(100)
    await drive_straight(150)
    await motor.run_for_degrees(ACC_LOW, 100, 1000)
    await drive_straight(-700, velocity=1000, acceleration=500)
    await motor.run_for_degrees(ACC_LOW, -100, 1000)


async def cross_field():
    """
    get from blue to red
    """
    await drive_straight(400, velocity=1000)
    await turn_to_angle(200)
    await drive_straight(270, velocity=1000)
    await turn_to_angle(-200)
    await drive_straight(600, velocity=1000)
    await turn_to_angle(-300)
    await drive_straight(500, velocity=1000)


async def blue_pickup():
    """
    pick up the stuff on the red side
    """
    await drive_straight(-350)
    await turn_to_angle(-600)
    await drive_straight(-200)
    await turn_to_angle(600)
    await drive_straight(500)


async def shark_and_stuff():
    """
    Coral, dump shark, dump sample, push coral
    by: nikesh and caleb
    """
    await drive_straight(810)
    await turn_to_angle(450)

    await motor.run_for_degrees(ACC_LOW, -250, 1000)
    await motor.run_for_degrees(ACC_LOW, 250, 1000)
    await turn_to_angle(-910)
    await drive_straight(-55)
    await motor.run_for_degrees(ACC_LOW, -250, 1000)
    await drive_straight(-175)
    await motor.run_for_degrees(ACC_LOW, 250, 1000)
    await drive_straight(100)
    await turn_to_angle(-400)
    await drive_straight(180)
    await drive_straight(-100)
    await turn_to_angle(-700)
    await drive_straight(650)
    await motor.run_for_degrees(ACC_LOW, -250, 1000)


async def lift_the_coral():
    """
    use the special coral attachment, by nikesh and caleb
    """
    await drive_straight(-300)
    await drive_straight(300)
    await drive_straight(-450)
    await drive_straight(-100, velocity=200, acceleration=500)
    await drive_straight(550)
    await turn_to_angle(-250)


async def push_shark():
    """
    push the shark home, but also try to poke the trident out
    """
    await drive_straight(650)
    await motor.run_for_degrees(ACC_LOW, 85, 500)
    await drive_straight(-100)
    await turn_to_angle(-145)
    await drive_straight(-100)
    await drive_straight(300, 1000, 5000)
    await drive_straight(-100)
    await turn_to_angle(170)
    await drive_straight(-700, 1000)
    await motor.run_for_degrees(ACC_LOW, 150, 500)


async def push_coral():
    """
    slowly push the coral out into the field
    """
    await drive_straight(150, 1000, 200)
    await drive_straight(-200, 1000, 500)


##### RUN LIST
# keeps a list of run name and run function
runs = [
    #("0", artbots),
    ("1", red_pickup),
    ("R", reset),
    ("2", send_sub_and_lanternfish),
    #("3", run4),# arrange 2x2
    #("4", run5),# flip up
    ("5", sonar_and_critter),
    ("C", cross_field),
    ("7", blue_pickup),
    ("8", shark_and_stuff),
    ("S", push_shark),
    ("P", push_coral),
    ("L", lift_the_coral),

    #("+", motor_test_up),
    #("-", motor_test_down),
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
