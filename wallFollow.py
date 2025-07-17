"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../../library')
import racecar_core
import racecar_utils as rc_utils
import math
import numpy as np

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

left_avg =0.0
right_avg = 0.0
last_offset = 0.0
integral_error = 0.0
angle = 0.0
speed = 0.0 
OFFSET_FULL_STEER = 60
GAIN = 2.5#2.4
angles=[]
errors = []
CurrFunc = np.polyfit([0,1], [0,0], deg =1)

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    speed = 0.0
    angle = 0.0
    rc.drive.set_speed_angle(speed, angle)
    CurrFunc = np.polyfit([0,1], [0,0], deg =1)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update_lidar():
    global left_avg
    global right_avg

    scan = rc.lidar.get_samples()
    left_avg = rc_utils.get_lidar_average_distance(scan,270,60)
    right_avg =rc_utils.get_lidar_average_distance(scan,90,60)
    if left_avg > 500 :
        left_avg = rc_utils.get_lidar_average_distance(scan,350,10)
        if left_avg > 300 : left_avg = 0
    if right_avg > 500 : 
        right_avg = rc_utils.get_lidar_average_distance(scan,10,10)
        if right_avg > 300 : right_avg = 0

  #  print(f'left avg: {left_avg} and right avg:{right_avg}')


def update():
    global last_offset
    global integral_error
    global angle 
    global speed 
    global OFFSET_FULL_STEER
    global GAIN
    global CurrFunc

    # angle control
    update_lidar()
    # ---------wall hug mode---------
    if (left_avg != 0 or  right_avg != 0):
        if left_avg == 0 : LEFT = 65
        else: LEFT = left_avg
        if right_avg == 0: RIGHT  = 65
        else : RIGHT = right_avg

        offset = RIGHT - LEFT
        offset_norm = offset / OFFSET_FULL_STEER
        P = math.tanh(GAIN *offset_norm)
        kp = 0.06
        kd = 0.002
        ki = 0.000#1
        integral_error += (rc.get_delta_time() * offset)
     #   P = kp * (offset)
        I = ki * integral_error
        D = kd * (offset - last_offset)
        last_offset = offset
        angle = P + I + D #+ CurrFunc(offset)

        angle = rc_utils.clamp(angle, -1, 1)
    else: angle  = 0.0

  #  CurrFunc= GainTuneFun(last_offset , angle)


    
    #Speed control
    speed_constant = 0.9
    if (left_avg != 0 and  right_avg != 0):
        if (left_avg/right_avg  <= 1): ratio =left_avg/right_avg
        elif (right_avg/left_avg <= 1):  ratio = right_avg/left_avg
        speed = ratio * speed_constant
    else:
        speed = 0.5

   # print(f'speed:{speed} and angle: {angle}')
    rc.drive.set_speed_angle(speed, angle)

def GainTuneFun(last_error, curr_angle):
    global angles
    global errors 
    if len(errors) > 20: errors.pop(0)
    if len(angles) > 20: angles.pop(0)

    errors.append(last_error)
    angles.append(curr_angle)

    coeffs = np.polyfit(errors,angles,deg =2)
    correction_func  = np.poly1d(coeffs)

    return correction_func
        

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    print(f'leftavg: { round(left_avg,2)} rightavg: {round(right_avg,2)}')
    print(f'speed {round(speed,2)} angle {round(angle,2)}')


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
