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
import os
import time

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'signmodel.tflite'
label_name = 'signs.txt'

model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 6

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
CROP_FLOOR = ((rc.camera.get_height()//3, 0), (rc.camera.get_height(), rc.camera.get_width()))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

last_error = 0
last_angle  = 0

curr_sign_obj = None
sign_class = None 
sign_bounds = [[0,0],[0,0]]
count = 0
last_class = None
sign_center = 0

left_avg =0.0
right_avg = 0.0
last_offset = 0.0
integral_error = 0.0
OFFSET_FULL_STEER = 140
GAIN = 0.7#0.6
angles=[]
errors = []
CurrFunc = np.polyfit([0,1], [0,0], deg =1)

Confirmed_sign = None

########################################################################################
# Functions
########################################################################################

# [FUNCTION] 

def update_object():
    global sign_class 
    global sign_bounds 
    global count
    global last_class
    global Confirmed_sign
    global curr_sign_obj
    global sign_center
    
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, CROP[0], CROP[1])

    if image is None:
        current_sign = None
        sign_bounds = [0,0]
    else:    
        #Loading model and labels
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()
        labels = read_label_file(label_path)
        inference_size = input_size(interpreter)
        #running model over image and getting objs 
        run_inference(interpreter, image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        if(objs.len() > 0):
            if(objs.len() == 1:
                curr_sign_obj = obj[0]
            else:
                print("BAD!-COME BACK TO")
        if curr_sign_obj is not None:
            if(curr_sign_obj.score > 0.55): sign_class = curr_sign.id
            if(sign_class == last_class):
                count += 1
                if count >= 5: 
                    Confirmed_sign = curr_sign_obj 
                    sign_class = Confirmed_sign.id 
                    bbox = obj.bbox.scale(scale_x, scale_y)
                    sign_bounds[0] = int(bbox.xmin), int(bbox.ymin)
                    sign_bounds[1] = int(bbox.xmax), int(bbox.ymax)
                    sign_center = (int(bbox.xmin) + int(bbox.xmax))//2
                    print(f'SIGN BOUNDS{sign_bounds}')
                    count = 0
            else: count = 0
            last_class = sign_class 

def update_lidar():
    global left_avg
    global right_avg

    scan = rc.lidar.get_samples()
    left_avg = rc_utils.get_lidar_average_distance(scan,310,20)
    right_avg =rc_utils.get_lidar_average_distance(scan,50,20)
    '''
    left_avg = rc_utils.get_lidar_average_distance(scan,310,20)
    right_avg =rc_utils.get_lidar_average_distance(scan,50,20)
    2181370DE4DF53F55F4B017A4D4940383F27AC76E65B3C179E85BEFAD008FE92 20/20
    '''
    if left_avg > 500 :
        left_avg = rc_utils.get_lidar_average_distance(scan,350,10)
        if left_avg > 300 : left_avg = 0
    if right_avg > 500 : 
        right_avg = rc_utils.get_lidar_average_distance(scan,10,10)
        if right_avg > 300 : right_avg = 0

# [FUNCTION] The start function is run once every time the start button is pressed
def remap_range(value, old_lower, old_upper, new_lower, new_upper):
    normalized = (value - old_lower) / (old_upper - old_lower)
    new_value = normalized * (new_upper - new_lower) + new_lower
    return new_value

def start():
    global speed
    global angle
# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    
    global sign_class 
    global sign_bounds 
    global Confirmed_sign
    global sign_center

    global last_offset
    global integral_error
    global angle 
    global speed 
    global OFFSET_FULL_STEER
    global GAIN
    global CurrFunc
    global left_avg
    global right_avg
    

    # Search for signs 
    update_object()
    update_lidar()

    #angle control
    if (Confirmed_sign is not None and sign_class == "Go Around"):
        offset = rc.camera.get_width() //2 # middle - 0
        kp = 0.005
        P = kp * (offset)
    else: 
        #else do line follower 
        offset = right_avg - left_avg
        offset_norm = offset / OFFSET_FULL_STEER
        P = 1 * math.tanh(GAIN *offset_norm)
       # kp = 0.0062
        kd = 0.12
        ki = 0.000#1
        integral_error += (rc.get_delta_time() * offset)
     #   P = kp * (offset)
        I = ki * integral_error
        D = kd * (offset - last_offset)
        last_offset = offset
    angle = P + I + D #+ CurrFunc(offset)

    angle = rc_utils.clamp(angle, -1, 1)

    #speed control
    if Confirmed_sign is not None:
        sign_class = Confirmed_sign.id
        if(sign_class == "Stop"):
            speed = 0 #maybe make negative needs testing
        if(sign_class == "Yield"):
            speed = 0.3
    else:
        speed = remap_range(speed, 0,1, 0.6, 1) # in wall follow this is speed = 1 but thats dangerous


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
    print(f'Angle {angle} and Speed {speed}')
    # #print(rc.camera.get_width())
    # # Print a line of ascii text denoting the contour area and x-position
    # if rc.camera.get_color_image() is None:
    #     # If no image is found, print all X's and don't display an image
    #     print("X" * 10 + " (No image) " + "X" * 10)
    # else:
    #     # If an image is found but no contour is found, print all dashes
    #     if contour_center is None:
    #         print("-" * 32 + " : area = " + str(contour_area))

    #     # Otherwise, print a line of dashes with a | indicating the contour x-position
    #     else:
    #         s = ["-"] * 32
    #         s[int(contour_center[1] / 20)] = "|"
    #         print("".join(s) + " : area = " + str(contour_area))

    #print("angle:", angle, "speed:", speed)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()