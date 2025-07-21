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
CROP_FLOOR = (((2 * rc.camera.get_height() //5), 0), (4 * rc.camera.get_height()//5, rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
'''
BLUE = ((90, 150, 50), (120, 255, 255))   # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  

RED=((20,146,46),(44,255,255)) #yellow
BLUE=((91,150,46),(122,255,255))

RED=((91,150,46),(122,255,255)) #Orange
BLUE #Green
'''
RED = ((1, 170, 180), (15, 255, 255)) #ORANGE
BLUE = ((36, 40, 89),  (68, 255, 255)) #GREEN




# Color priority: Red >> Green >> Blue

state = None

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
color = None #true is red, false is blue, none is neither
lastcolor = None
lastarea = 0

circular_dist = 70

kp = -0.05


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
                    color = "BLUE"
                    print("PLEASE HELP")
                    contour_center = rc_utils.get_contour_center(contourB)
                    contour_area = rc_utils.get_contour_area(contourB)
                    rc_utils.draw_contour(image,contourB)
                    rc_utils.draw_circle(image, contour_center)

                elif contourB is None:
                    contour = contourR
                    color = "RED"
                    contour_center = rc_utils.get_contour_center(contourR)
                    contour_area = rc_utils.get_contour_area(contourR)
                    rc_utils.draw_contour(image,contourR)
                    rc_utils.draw_circle(image, contour_center)
                else:
                    contourB_area = rc_utils.get_contour_area(contourB)
                    contourR_area = rc_utils.get_contour_area(contourR) 
                    print(f'Barea {contourB_area} Rarea {contourR_area}')
                    if contourR_area > contourB_area:
                        contour = contourR
                        color = "RED"
                        contour_center = rc_utils.get_contour_center(contourR)
                        contour_area = rc_utils.get_contour_area(contourR)
                        rc_utils.draw_contour(image,contourR)
                        rc_utils.draw_circle(image, contour_center)
                    else:
                        contour = contourB
                        color = "BLUE"
                        print("HIHI")
                        contour_center = rc_utils.get_contour_center(contourB)
                        contour_area = rc_utils.get_contour_area(contourB)
                        rc_utils.draw_contour(image,contourB)
                        rc_utils.draw_circle(image, contour_center)                        


        # Display the image to the screen only if display is available
        if hasattr(rc.display, "show_color_image"):
            rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global state
    global speed,angle
    global color,contour_area,lastarea,lastcolor
    global kp

    # Initialize variables
    state = "searching"

    speed = 0
    angle = 0

    color = None
    contour_area = None
    lastarea = None
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

def certain_dist(scan, dist):
    # Option 1: Set invalid readings to 0 (recommended)
    new_scan = scan.copy()
    for i in range(len(new_scan)):
        if not (0 < new_scan[i] <= dist):
            new_scan[i] = 0
    return new_scan

def updateWRIE():
    global color,contour_area,lastarea,lastcolor
    update_contour()
    scan = rc.lidar.get_samples()
    # print("----------------------------------------------------------")
    scan = certain_dist(scan, 70)
    # print("----------------------------------------------------------")
    # front = rc_utils.get_lidar_average_distance(scan,0,55)
    # skinny_front = rc_utils.get_lidar_average_distance(scan,0,20)
    Rside = rc_utils.get_lidar_average_distance(scan,315,50)
    Bside = rc_utils.get_lidar_average_distance(scan,45,50)

    print(f'RSIDE: {Rside} and BSIDE: {Bside}')

   # print(f'color {color} area {contour_area}')
    print
cnt = 0
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed,angle
    global color,contour_area,lastarea,lastcolor
    global circular_dist
    global kp,state, cnt



    CURRENTCONE = "NONE"

    LASTSTATE = "NONE"
    CONTOUR_CONE = 2500 #2700



    rc.drive.set_max_speed(0.6) #0.45
    # Search for contours in the current color image
    CURRENTCONE = color
    update_contour()
    
    scan = rc.lidar.get_samples()
    # print("----------------------------------------------------------")
    scan = certain_dist(scan, 50)
    Rside = (scan,315,50)
    Bside = (scan,45,50)
    
    # print("----------------------------------------------------------")
    # front = rc_utils.get_lidar_average_distance(scan,0,55)
    # skinny_front = rc_utils.get_lidar_average_distance(scan,0,20)
    Rside = rc_utils.get_lidar_average_distance(scan,315,50)
    Bside = rc_utils.get_lidar_average_distance(scan,45,50)
    speed = 0.3
    dt = rc.get_delta_time()
    if contour_center is not None:
        if state == "searching":
            cnt = 0
            speed = 0.4
            print("----------SEARCHING----------")
            # print("front: ", front, "color: ", color)
            if(lastcolor is None): # or color is None
                #print("straight")
                angle = 0
            else:
                state ="pass"
            if(contour_area > CONTOUR_CONE and color is not None):
                print("area: ", contour_area)
                state = "approaching"
        elif state == "approaching":
            print("----------APPROACHING----------")
            if color is None: state= "pass"
            else:
                print( "color: ", color)
                setpoint = rc.camera.get_width()//2
                print(f'contourcenter {contour_center[1]} setpoint {setpoint}')
                if(color == "RED"):
                    #setpoint = rc.camera.get_width()//10
                    setpoint = 0
                elif(color == "BLUE"):
                    print("HELLLOLOOOOO")
                    #setpoint = (4*rc.camera.get_width())//5
                    setpoint = rc.camera.get_width()
                error = (setpoint - contour_center[1])
                speed = 0.2
                kp = -0.03
                angle = kp * error
                angle = clamp(angle,-1,1) 

            cnt += dt
            if(color == "RED" and color != lastcolor and color is not None and Rside ==0) and cnt > 0.25: 
                cnt = 0
                state = "pass"
            if(color == "BLUE" and color != lastcolor and color is not None and Bside == 0) and cnt > 0.25: 
                cnt = 0
                state = "pass"

        elif state == "pass":
            cnt = 0
            speed = 0.4
            kp = -0.01
            print("----------PASS----------")
            print(f'contour area: { contour_area}')
            print( "color: ", color)
            if(contour_area > CONTOUR_CONE): state ="approaching"
            else:
                setpoint = rc.camera.get_width()//2
                print(f'contourcenter {contour_center[1]} setpoint {setpoint}')
                error = (setpoint - contour_center[1])
                speed = 0.1
                kp = -0.03
                angle = kp * error
                angle = clamp(angle,-1,1) 
            # if(lastcolor == "RED"):
            #     print("RED")
            #     setpoint = circular_dist
            #     error = setpoint - Rside
            #     angle = kp * error
            #     angle = clamp(angle, -1, 1)
            # elif(lastcolor == "BLUE"):
            #     print("BLUE")
            #     setpoint = circular_dist
            #     error = Bside - setpoint
            #     angle = kp * error
            #     angle = clamp(angle, -1, 1)
            # if(skinny_front < 300 and skinny_front != 0):
            #     lastcolor = None
            #     state = "searching"
                
    if color is not None: lastcolor = color
    # if Rside < 20 or Lside < 20:
    #     angle = 0
    speed = remap_range(speed, 0, 0.4, 0.7, 0.9)
    rc.drive.set_speed_angle(speed, angle)
    print("speed: ", round(speed, 2), "angle: ", round(angle, 2))

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

