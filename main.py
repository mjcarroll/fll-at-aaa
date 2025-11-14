######### IMPORTS
import color
import color_sensor
import motor
import motor_pair

from hub import button
from hub import light
from hub import light_matrix
from hub import motion_sensor
from hub import port

import math
import runloop
import time

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
motor_pair.pair(PAIR_IDX, LEFT_MOTOR, RIGHT_MOTOR)

######### HELPERS
async def run_rate(func, rate_ms: int):
    """
    Run a function at a fixed rate
    """
    rate_us = rate_ms * 1000
    next_run_time_us: int = time.ticks_us()
    last_run_time_us: int = next_run_time_us
    while True:
        # 1. Get the current time in microseconds
        current_time_us: int = time.ticks_us()

        # 2. Calculate how long to sleep in microseconds
        #    We check the difference between our target 'next_run_time_us'
        #    and the 'current_time_us'.
        sleep_time_us: int = time.ticks_diff(next_run_time_us, current_time_us)

        # 3. Convert sleep time to milliseconds for the sleep function.
        #    We must sleep in 'ms', so we get the 'floor' value.
        sleep_time_ms: int = sleep_time_us // 1000

        # 4. If we are not behind schedule (sleep_time_ms > 0),
        #    then sleep for that precise calculated amount.
        #    If sleep_time_ms is 0 or negative, we're running late,
        #    so we skip sleeping and run the loop immediately.
        if sleep_time_ms > 0:
            await runloop.sleep_ms(sleep_time_ms)

        # 5. Set the *next* run time, 'rate_us' from the *previous*
        #    target. This is key: we add to 'next_run_time_us',
        #    not the 'current_time_us'. This ensures the loop
        #    averages the target rate even if one loop is slightly delayed.
        next_run_time_us = time.ticks_add(next_run_time_us, rate_us)

        # --- Call the provided function ---
        # Assumes 'func' is a synchronous function
        current_run_time_us: int = time.ticks_us()
        delta_time_us: int = time.ticks_diff(current_run_time_us, last_run_time_us)
        last_run_time_us = current_run_time_us
        ret = func(int(current_run_time_us / 1000), int(delta_time_us / 1000))
        if not ret:
            break

class GyroDriveStraight:
    # Error times milliseconds, normalize before use
    error_accum: int
    prev_error: int
    setpoint: int

    windup:int = 1000
    Kp: float = 0.3
    Ki: float = 0.5
    Kd: float = 0.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.error_accum = 0
        self.prev_error = 0
        self.setpoint = 0
        self.reverse = False
        motor.reset_relative_position(LEFT_MOTOR, 0)
        motor.reset_relative_position(RIGHT_MOTOR, 0)
        motion_sensor.reset_yaw(0)

    def distance_travelled(self) -> int:
        L = abs(motor.relative_position(LEFT_MOTOR))
        R = abs(motor.relative_position(RIGHT_MOTOR))
        return int((L + R) / 2)

    def _error_sum(self):
        if self.error_accum > self.windup:
            self.error_accum = self.windup
        elif self.error_accum < -self.windup:
            self.error_accum = -self.windup
        return self.error_accum / 1000

    def _update(self, z: int, dt: int):
        print("_update", z, dt)
        error = self.setpoint - z
        self.error_accum = self.error_accum + error * dt
        if dt > 0:
            d_error = (error - self.prev_error) / dt
        else:
            d_error = 0
        output = self.Kp * error + self.Ki * self._error_sum() + self.Kd * d_error
        self.prev_error = error
        print("Corr: ", self.Kp * error, self.Ki * self._error_sum(), self.Kd * d_error, output)
        return output

    def _tick(self, t, dt):
        if abs(self.distance_travelled()) > abs(self.degrees_to_move):
            print("drive straight (done): ", self.distance_travelled())
            return False
        curHeading = motion_sensor.tilt_angles()[0]
        if self.reverse:
            curHeading = -curHeading
        correction = self._update(curHeading, dt)
        print(self.velocity - int(self.velocity * correction / 100), self.velocity + int(self.velocity * correction / 100))
        motor_pair.move_tank(PAIR_IDX,
            self.velocity - int(self.velocity * correction / 100),
            self.velocity + int(self.velocity * correction / 100)
        )
        return True

    async def run(self, distance: int, velocity: int, dt: int = 25):
        self.reset()
        self.degrees_to_move = int((3600 * distance) / WHEEL_CIRC)
        self.velocity = velocity

        if self.degrees_to_move < 0:
            self.reverse = True
            self.velocity = -self.velocity
        self.dt = dt
        print("drive straight: ", distance, self.degrees_to_move, self.velocity, self.dt)
        await run_rate(self._tick, 25)
        motor_pair.stop(PAIR_IDX)


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


gds = GyroDriveStraight()
async def gyro_drive_straight(
    target_distance: int,
    velocity: int = 500
):
    await gds.run(target_distance, velocity)
    await runloop.sleep_ms(10)


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
    await gyro_drive_straight(610, velocity=600)

    motor_pair.move(PAIR_IDX, 100, velocity=300)
    await runloop.sleep_ms(100)

    motor_pair.stop(PAIR_IDX)
    
    await motor.run_for_degrees(ACC_HIGH, flag_turn, 500)
    motor.run_for_degrees(ACC_HIGH, -flag_turn, 500)
    motor.run_for_degrees(ACC_LOW, -300, 500)
    await drive_straight(-150, velocity=450)
    motor.run_for_degrees(ACC_LOW, 300, 500)
    await drive_straight(-500, velocity=1000)


    return
    await motor.run_for_degrees(ACC_HIGH, flag_turn, 500)
    await runloop.sleep_ms(100)
    await drive_straight(-100)
    await motor.run_for_degrees(ACC_LOW, -240, 500)
    await drive_straight(-150, velocity=450)
    motor.run_for_degrees(ACC_LOW, 240, 500)
    motor.run_for_degrees(ACC_HIGH, -flag_turn, 500)
    await drive_straight(-300, velocity=1000)


async def surface_brushing_map_reveal():
    """
    M02: 30 (10 pt/obj)
    M01: 30 (10 pt/obj + 10)


    LINE UP [RED]: 10 squares from left (left corner robot)
    """
    await gyro_drive_straight(675, velocity=500)
    await runloop.sleep_ms(100)
    await turn_to_angle(-420)
    await gyro_drive_straight(140, velocity=300)
    await runloop.sleep_ms(10)
    await motor.run_for_degrees(ACC_HIGH, 300, 500) # M02 (Map Reveal)
    await drive_straight(-140)
    await turn_to_angle(480)
    await drive_straight(-710, velocity=1000)
    return 

async def cross_field():
    motor.run_for_degrees(ACC_HIGH, 180, 250)
    await gyro_drive_straight(820)
    await runloop.sleep_ms(50)
    await turn_to_angle(900)
    await motor.run_for_degrees(ACC_HIGH, -180, 250)
    await drive_straight(180)
    # We are now under the minecart
    await motor.run_for_degrees(ACC_HIGH, 400, 250)
    await motor.run_for_degrees(ACC_HIGH, -200, 250)
    motor.run_for_degrees(ACC_HIGH, -200, 250)
    await turn_to_angle(490)
    await drive_straight(200)

    # Lifting the statue
    await motor.run_for_degrees(ACC_HIGH, 100, 100)
    await turn_to_angle(-100)
    await motor.run_for_degrees(ACC_HIGH, 300, 500)
    await drive_straight(-100)
    await turn_to_angle(-460)

    await gyro_drive_straight(670)
    await runloop.sleep_ms(50)
    await turn_to_angle(450)

    #await motor.run_for_degrees(ACC_HIGH, 400, 1000)
    #await motor.run_for_degrees(ACC_HIGH, -80, 250)
    await drive_straight(150, velocity=250)
    await drive_straight(-150, velocity=500)
    await turn_to_angle(-450)
    await drive_straight(300, velocity=1000)
    await turn_to_angle(450)
    await drive_straight(800, velocity=1000)

async def who_lived_and_forge():
    """
    M06: 30 (10 pt/rock)
    M05: 30
    M07: 30


    LINE UP [BLUE]: 2 squares from left (left corner robot)
    """
    await gyro_drive_straight(660, velocity=600)
    await runloop.sleep_ms(100)
    await turn_to_angle(450)
    await drive_straight(30)
    color_check = True
    while(color_check):
        await drive_straight(5)
        if color_sensor.color(SNS_LEFT) is color.BLACK and color_sensor.color(SNS_RIGHT) is color.BLACK:
            color_check = False
    await drive_straight(-20)
    await turn_to_angle(-700, sleep_ms=200) # dump rocks M06 (Forge)
    await drive_straight(50)
    await turn_to_angle(-170) # flip M05 (Who Lived Here?)
    await drive_straight(-50)
    await turn_to_angle(-470) # move rocks into home area
    await drive_straight(-410)
    await turn_to_angle(640) # set up M07 (Heavy Lifting)
    await motor.run_for_degrees(ACC_LOW, -125, 500) # drop armNEED TO DOUBLE CHECK
    await drive_straight(30)
    await turn_to_angle(-100)
    await motor.run_for_degrees(ACC_LOW, 125, 100) # pick up millstone

    await turn_to_angle(100)
    await drive_straight(-1000, velocity= 1000) # return home
    return

async def tip_the_scales():
    """
    M10: 30
    LINE UP [BLUE]: 9 squares from left on launch/home border (left corner robot)
    """
    await gyro_drive_straight(-190)
    await turn_to_angle(900)
    await gyro_drive_straight(600)
    await turn_to_angle(-900)
    await drive_straight(-150) # tip scale
    await drive_straight(150) # remove pan
    await turn_to_angle(-700)
    await drive_straight(800, velocity=1000) # return home
    return

async def forum():
    """
    M14: 25 (5 pt/obj)


    LINE UP [BLUE]: 3 black lines from left (left corner robot), 2 black lines on arc (right cage wall)
    """
    await gyro_drive_straight(-1075, velocity= 700)
    await gyro_drive_straight(200, velocity= 1500) # backup to not touch artifacts
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
    #("0", artbots),
    # From the RED SIDE
    ("1", boat),
    ("2", surface_brushing_map_reveal),
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
