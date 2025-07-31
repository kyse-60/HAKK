"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: RACECAR Team 7

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""
# things to fix
########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 50

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((rc.camera.get_height()//6, 0), (4*rc.camera.get_height()//6, rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((90, 150, 50), (120, 255, 255))  # The HSV range for the color blue
# GREEN = ((35, 50, 50), (75, 255, 255))  # The HSV range for the color green
RED = ((0, 50, 50), (10, 255, 255)) # The HSV range for the color red
# ORANGE = ((9, 54, 169), (90, 255, 255)) # The HSV range for the color red
ORANGE = ((4, 75, 169), (15, 255, 255)) # The HSV range for the color red
# ORANGE = ((9, 68, 169), (15, 255, 255)) # The HSV range for the color red
GREEN = ((30, 58, 57), (80, 255, 255))
# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (ORANGE, )

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cntr = 0
last_error = 0
last_angle  = 0
past_five = []



counter = 0
# kp = -0.0058   
# kd = -0.0006 #-0.0005 works but oscilates on straight
########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        for color in COLOR_PRIORITY:
            contours = rc_utils.find_contours(image, color[0], color[1])

            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                    contour_center = rc_utils.get_contour_center(contour)
                    contour_area = rc_utils.get_contour_area(contour)

                    rc_utils.draw_contour(image,contour)
                    rc_utils.draw_circle(image, contour_center)
                    break 

# [FUNCTION] The start function is run once every time the start button is pressed
def remap_range(value, old_lower, old_upper, new_lower, new_upper):
    normalized = (value - old_lower) / (old_upper - old_lower)
    new_value = normalized * (new_upper - new_lower) + new_lower
    return new_value

def start():
    global speed
    global angle
    global kp 
    global kd 
    global counter
    
    #kp = -0.0058    #-0.06 osolates like crazy, not in a bangbangy kinda way but not staying on the line                     #0.05 works but dive #0.007
    #kd = -0.0004
    #with open("log.txt", "a") as f:
    #    print("\n\nNEW\n\n", file=f)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cntr
    global last_error
    global last_angle, past_five
    global kp 
    global kd 
    global counter

    global contour_area
    global contour_center
    # Search for contours in the current color image
    update_contour()


    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle

    if contour_center is not None:
        setpoint = rc.camera.get_width()//2
        error = (setpoint - contour_center[1])
        kp = -0.0058   
        kd = -0.0000 #-0.0007
        # kp = -0.0055    #-0.06 osolates like crazy, not in a bangbangy kinda way but not staying on the line                     #0.05 works but dive #0.007
        # kd = -0.0004#-0.0065 # -0.006, -0.009
        past_five.append(error)
        dterm = 0
        if len(past_five) > 5:
            past_five.pop(0)
        if len(past_five) == 5:
             e0, e1, e2, e3, e4 = past_five
             # apply the five-point backward-difference formula
             dterm = (25*e4- 48*e3+ 36*e2- 16*e1+  3*e0) / (12)
        angle = kp * error + kd*dterm
        angle = max(-1, min(1, angle))
        cntr = 0

        print("speed: ", round(speed, 2), " angle: ", round(angle, 2), " lastangle: ", round(last_angle, 2), " error: " , round(error, 2), " dterm :", round(dterm, 2), " contour_area: ", round(contour_area, 2), " contour_center: ", contour_center)


        
        if(last_angle == angle):
            counter += 1
            if(counter >= 10):
                print("===============================")
                print("contour_area: ", round(contour_area, 2), " contour_center: ", contour_center)
                print("===============================")
        else:
            counter = 0

        
        last_error = error 
        last_angle = angle 
        
    else: 
        print("dont see anything")
        cntr += 1
        if cntr > 5:
            angle = last_angle 

    if angle < 0.1:
        speed = remap_range(angle, -1, 0, 0.6, 0.9)
    elif angle > -0.1:
        speed = remap_range(angle, 0, 1, 0.9, 0.6)
    else: 
        speed = 1


    # Use the triggers to control the car's speed
    # InP = rc.controller.Button.A()
    # DeP = rc.controller.Button.B()

    # InD = rc.controller.Button.X()
    # DeD = rc.controller.Button.Y()

    # if InP: kp -= 0.0001
    # if Dep: kp += 0.0001
    # if InD: kd -= 0.0001
    # if DeD: kd += 0.0001
    
    # speed = 0.8 if angle == 0 else 0.5
    # with open("log.txt", "a") as f:
    #     if rt:
    #         print("angle:", angle, "speed:", speed, file=f)
    # print("speed: ", speed, "angle: ", angle, "error : " , error, "dterm :", dterm)
    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global angle
    global speed
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # print(f'kp {kp} and kd {kd}')
    #print(rc.camera.get_width())
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))

    #print("angle:", angle, "speed:", speed)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()