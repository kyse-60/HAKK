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
RED = ((0, 50, 50), (10, 255, 255))  

# Color priority: Red >> Green >> Blue

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

color = None #true is red, false is blue, none is neither

lastcolor = None

lastarea = 0

kp = -0.003


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    global color

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
            color = None

        # Display the image to the screen only if display is available
        if hasattr(rc.display, "show_color_image"):
            rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    # Initialize variables
    speed = 0
    angle = 0

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
        speed = 0.5
        '''
        if(contour_area == 0 or lastarea == contour_area):
            print("NOTHING IS SHOWING", lastcolor)
            if(lastcolor == True):
                angle = -1
            elif(lastcolor == False):
                angle = 1
        '''

        if(abs(contour_area - lastarea) > 5000 or contour_area == 0 or (lastarea > 663.0 and lastarea == contour_area)):

            
            back =rc_utils.get_lidar_average_distance(scan,180,200)
            print("-------BACK-----: ", back)

            print("CHANGE")

            if(contour_area - lastarea > 5000):
                print("contour_area - lastarea > 5000")
                
            if(contour_area == 0):
                print("contour_area == 0")
            if(lastarea == contour_area):
                print("lastarea == contour_area")
                print(contour_area)
            
            
            if(lastcolor == True):
                angle = remap_range(back, 0, 400, 0.1, -1)
            elif(lastcolor == False):
                angle = remap_range(back, 0, 400, -0.1, 1)
            
        elif(color == True):
            '''

            print("RED")
            setpoint = 100
            REDside =rc_utils.get_lidar_average_distance(scan,-45, 90)
            print("------------REDSIDE: ", REDside)
            error = setpoint - REDside
            #kp = -0.005 # orev value -0.003125
            angle = kp * error
            angle = clamp(angle,-1,1)
            lastcolor = True
            '''
            print("RED")
            setpoint = rc.camera.get_width()//10
            setpoint = 0
            error = setpoint - contour_center[1]
            #kp = -0.005 # orev value -0.003125
            angle = kp * error
            angle = clamp(angle,-1,1)
            lastcolor = True

            
        elif(color == False): #BLUE
            print("BLUE")
            setpoint = (9*rc.camera.get_width())//10
            setpoint = rc.camera.get_width()
            error = setpoint - contour_center[1]
            #kp = -0.005 # orev value -0.003125
            angle = kp * error
            angle = clamp(angle,-1,1) 
            lastcolor = False

        else:
            angle = 0
        lastarea = contour_area
        #print("I literally dont know whats happening")
        print("speed: ", round(speed, 2), " angle: ", round(angle, 2))           

        #BLUE
    else: #nothin
        print("NOTHING IS SHOWING")
        if(lastcolor == True):
            angle = -1
        elif(lastcolor == False):
            angle = 1


        speed = 0.5
        angle = 0
    # Use the triggers to control the car's speed
    '''
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    '''
    
    if color == True:
        angle *= 1.05
    angle = clamp(angle,-1,1) 
    rc.drive.set_speed_angle(speed, angle)

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