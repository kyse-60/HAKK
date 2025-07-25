"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

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
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((230, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((90, 150, 50), (120, 255, 255))   # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  

# Color priority: Red >> Green >> Blue

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

color = None #true is red, false is blue, none is neither

lastcolor = None

lastarea = 0

kp = -0.01


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    global color

    color = None
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        contoursR = rc_utils.find_contours(image, RED[0], RED[1])
        contoursB = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        contourR = rc_utils.get_largest_contour(contoursR, MIN_CONTOUR_AREA)
        contourB = rc_utils.get_largest_contour(contoursB, MIN_CONTOUR_AREA)

        if contourR is not None or contourB is not None:
                if contourR is None:
                    contour = contourB
                    color = False
                    contour_center = rc_utils.get_contour_center(contourB)
                    contour_area = rc_utils.get_contour_area(contourB)
                    rc_utils.draw_contour(image,contourB)
                    rc_utils.draw_circle(image, contour_center)

                elif contourB is None:
                    contour = contourR
                    color = True
                    contour_center = rc_utils.get_contour_center(contourR)
                    contour_area = rc_utils.get_contour_area(contourR)
                    rc_utils.draw_contour(image,contourR)
                    rc_utils.draw_circle(image, contour_center)
                else:
                    contourB_area = rc_utils.get_contour_area(contourB)
                    contourR_area = rc_utils.get_contour_area(contourR) 
                    if contourR_area > contourB_area:
                        contour = contourR
                        color = True
                        contour_center = rc_utils.get_contour_center(contourR)
                        contour_area = rc_utils.get_contour_area(contourR)
                        rc_utils.draw_contour(image,contourR)
                        rc_utils.draw_circle(image, contour_center)
                    else:
                        contour = contourB
                        color = False
                        contour_center = rc_utils.get_contour_center(contourB)
                        contour_area = rc_utils.get_contour_area(contourB)
                        rc_utils.draw_contour(image,contourB)
                        rc_utils.draw_circle(image, contour_center)                        

        else:
            color = None

        # Display the image to the screen only if display is available
        if hasattr(rc.display, "show_color_image"):
            rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    global color
    global contour_area
    global lastarea

    global lastcolor

    global kp
    # Initialize variables
    speed = 0
    angle = 0

    color = None
    contour_area = 0
    lastarea = 0
    lastcolor = None

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    print("-------------------------------------START-----------------------------------------")


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def clamp(value :float, min:float , max :float ) -> float:
    if value <min:
        return min 
    if value > max:
        return max 
    else: return value

def remap_range(val: float, old_min:float , old_max : float, new_min :float, new_max :float) -> float:
    old_range = old_max - old_min 
    new_range = new_max - new_min
    return new_range * (float( val - old_min) / float(old_range)) + new_min

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle

    global color
    global contour_area
    global lastarea

    global lastcolor

    global kp
    rc.drive.set_max_speed(0.45)
    # Search for contours in the current color image
    update_contour()
    scan = rc.lidar.get_samples()
    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        front = rc_utils.get_lidar_average_distance(scan,0,40)

        Rside = rc_utils.get_lidar_average_distance(scan,270,90)
        Bside = rc_utils.get_lidar_average_distance(scan,90,90)

        back = rc_utils.get_lidar_average_distance(scan,180,60)

        everywhere = rc_utils.get_lidar_average_distance(scan,180,180)
        print("-----ALL LIDAR VALUES: ", round(everywhere, 2))



        if(color == None):
            angle = 0
        if(front < 90 and front != 0 and color is not None):
            print("---------APPROACHING---------")
            if(color == True): #RED
                print("RED")
                angle = 1
                lastcolor = color
            if(color == False): #BLUE
                print("BLUE")
                angle = -1
                lastcolor = color
        elif(color == None and everywhere < 100): # and back < 100
            print("---------PASS---------")
            if(lastcolor == True): #RED
                if(Rside == 0):
                    angle = 0
                print("RED: ", everywhere)
                setpoint = 20
                error = everywhere - setpoint
                angle = kp * error
                angle = clamp(angle,-1,1) 
            if(lastcolor == False): #BLUE
                print("BLUE")
                setpoint = 50
                error = setpoint - Bside
                angle = kp * error
                angle = clamp(angle,-1,1)  
        elif(abs(lastarea - contour_area) > 1000):
            print("-------GOING BACK--------")
            if(lastcolor == True): #RED
                print("RED")
                angle = -1
            if(lastcolor == False): #BLUE
                print("BLUE")
                angle = 1

    print("speed: ", round(speed, 2), "angle: ", round(angle, 2))
    rc.drive.set_speed_angle(0.5, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
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


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()