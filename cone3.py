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
CROP_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((90, 150, 50), (120, 255, 255))   # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  

# Color priority: Red >> Green >> Blue

state = None

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
color = "None" #true is red, false is blue, none is neither
lastcolor = None
lastarea = 0

circular_dist = 30

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
    global state
    global speed,angle
    global color,contour_area,lastarea,lastcolor
    global kp

    # Initialize variables
    state = "searching"

    speed = 0
    angle = 0

    color = "None"
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

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed,angle
    global color,contour_area,lastarea,lastcolor
    global circular_dist
    global kp,state



    CURRENTCONE = "NONE"

    LASTSTATE = "NONE"



    rc.drive.set_max_speed(0.45)
    # Search for contours in the current color image
    CURRENTCONE = color
    update_contour()
    scan = rc.lidar.get_samples()
    front = rc_utils.get_lidar_average_distance(scan,0,45)
    skinny_front = rc_utils.get_lidar_average_distance(scan,0,10)
    Rside = rc_utils.get_lidar_average_distance(scan,270,45)
    Bside = rc_utils.get_lidar_average_distance(scan,90,45)
    speed = 0.3
    if contour_center is not None:
        if state == "searching":
            if(front<60):
                    print("skip")
                    state = "found"
            print("----------SEARCHING----------")
            lastcolor = None
            '''
            if(lastcolor == "None"):
                print(lastcolor)
                angle = 0
            else:
                state = "pass"
            '''
            

            angle = 0
            if(front < 120): # and contour_area > 1000
                print("front: ", front)
                state = "approaching"
        elif state == "approaching":
            print("----------APPROACHING----------")
            print("color: ", color)
            if(front<60):
                    print("skip")
                    state = "found"
            speed = 0.1
            
            setpoint = rc.camera.get_width()//2
            error = (setpoint - contour_center[1])
            kp = -0.005
            angle = kp * error
            angle = clamp(angle,-1,1) 
            #if(lastarea is not None and abs(lastarea - contour_area) > 1500) or lastcolor != color:
            #    state = "pass"
            
            #if(lastcolor is not None and lastcolor != color):
            #    state = "pass"
            lastarea = contour_area
            if color is not None and color != "None": lastcolor = color

            if(front < 100): # and color != "None" and contour_area > 15000
                print("front: ", front)
                state = "found"
        elif state == "found":
            speed = 0.2
            print("----------FOUND----------")
            print("front: ", front, "contour area: ", contour_area)
            if(color == "RED"):
                angle = 1
            elif(color == "BLUE"):
                angle = -1
            if color is not None and color != "None": lastcolor = color
            print(color)
            if(front == 0 and(Rside > 20 or Bside > 20) ): #bruh abs(lastarea - contour_area) > 7000 or 
                state = "pass"
        elif state == "pass":
            kp = -0.05
            print("----------PASS----------")
            print(lastcolor)
            if(lastcolor == "RED"):
                print("RED")
                setpoint = circular_dist
                error = setpoint - Rside
                angle = kp * error
                angle = clamp(angle, -1, 1)
                angle = -1
            elif(lastcolor == "BLUE"):
                print("BLUE")
                setpoint = circular_dist
                error = Bside - setpoint
                angle = kp * error
                angle = clamp(angle, -1, 1)
                angle = 1
            if(skinny_front < 140 and skinny_front != 0):

                state = "searching"

    if(LASTSTATE == state and lastcolor != color):
        print("LASTSTATE: ", LASTSTATE, " STATE: ", state)
        print("lastcolor", lastcolor ,"color ", color )
        print("please stop this madness")
        
    
    LASTSTATE = state

    
    
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




'''
def get_lidar_average_distance_filtered(scan, angle, window, max_distance=100):
    """
    Gets the average distance of lidar readings within a specific angular window,
    but only considers readings within max_distance.
    
    Args:
        scan: lidar scan data
        angle: center angle to look at
        window: angular window (total degrees to consider)
        max_distance: maximum distance to consider (ignores readings beyond this)
    
    Returns:
        Average distance of valid readings within the window and distance limit
    """
    # Get all samples in the angular window
    samples = rc_utils.get_lidar_closest_point(scan, angle, window)
    
    # Filter out readings beyond max_distance and invalid readings (0)
    valid_samples = []
    for i in range(len(scan)):
        # Calculate angle for this sample
        sample_angle = i * 360 / len(scan)
        
        # Check if this sample is within our angular window
        angle_diff = abs(sample_angle - angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
            
        if angle_diff <= window / 2:
            distance = scan[i]
            if 0 < distance <= max_distance:  # Valid reading within our distance limit
                valid_samples.append(distance)
    
    # Return average of valid samples, or 0 if no valid samples
    if valid_samples:
        return sum(valid_samples) / len(valid_samples)
    else:
        return 0

# Alternative simpler version if you just want to cap the distance
def get_lidar_average_distance_capped(scan, angle, window, max_distance=100):
    """
    Gets the average distance but caps any reading above max_distance to max_distance
    """
    # Get the normal average
    avg_distance = rc_utils.get_lidar_average_distance(scan, angle, window)
    
    # Cap it to max_distance
    if avg_distance > max_distance:
        return max_distance
    else:
        return avg_distance

# Usage in your update function:
def update():
    # ... existing code ...
    
    scan = rc.lidar.get_samples()
    
    # Use the filtered version - only consider readings within 100cm
    front = get_lidar_average_distance_filtered(scan, 0, 45, max_distance=100)
    skinny_front = get_lidar_average_distance_filtered(scan, 0, 10, max_distance=100)
    Rside = get_lidar_average_distance_filtered(scan, 270, 45, max_distance=100)
    Bside = get_lidar_average_distance_filtered(scan, 90, 45, max_distance=100)
'''