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
import csv

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'carmodel_edgetpu.tflite'
label_name = 'cars.txt'

model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 6

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "./library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

last_error = 0
last_angle  = 0

curr_obj = None
sign_class = None 
sign_bounds = [[0,0],[0,0]]
count = 0 
sign_center = 0

left_avg =0.0
right_avg = 0.0
last_offset = 0.0
integral_error = 0.0
angles=[]
errors = []
CurrFunc = np.polyfit([0,1], [0,0], deg =1)
start_time = 0

Confirmed_sign = None
CSV_FILENAME = 'data_log.csv'
CSV_HEADERS = ['time', 'id', 'reading']


########################################################################################
# Functions
########################################################################################

# [FUNCTION] 
def startCSVfile():
    # Check if the file already exists.
    if os.path.exists(CSV_FILENAME):
        # If it exists, remove it to start fresh.
        os.remove(CSV_FILENAME)
        print(f"Removed existing file: '{CSV_FILENAME}'")

    # Create a new file and write the headers.
    # 'w' stands for write mode, which creates a new file.
    # newline='' is important to prevent blank rows from being added.
    with open(CSV_FILENAME, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(CSV_HEADERS)
    print(f"Successfully created '{CSV_FILENAME}' with headers.")    

def CSVMAKE(makeCSV):
    global speed 
    global angle 


    if makeCSV : 
        timestamp = time.time() - start_time
        #row_distance = [timestamp,1,scan_distance]
        #row_target_angle = [timestamp, 2, scan_angle]
        row_angle = [timestamp, 3, angle]
        row_speed = [timestamp, 4, speed]

        with open(CSV_FILENAME, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # writer.writerow(row_distance)
            # writer.writerow(row_target_angle)
            writer.writerow(row_angle)
            writer.writerow(row_speed)

def update_object():
    global sign_class 
    global sign_bounds 
    global count
    global Confirmed_sign
    global curr_obj
    global sign_center
    
    image = rc.camera.get_color_image()

    if image is None:
        current_sign = None
        sign_bounds = [0,0]
    else:    
        #Loading model and labels
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()
        labels = read_label_file(label_path)
        inference_size = input_size(interpreter)
        scale_x, scale_y = rc.camera.get_width() / inference_size[0], rc.camera.get_height() / inference_size[1]
        #print(inference_size)
        image = cv.resize(image, inference_size)
        run_inference(interpreter, image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        if(len(objs) > 0):
            print("detecting")
            if(len(objs) == 1):
                curr_obj = objs[0]
            else:
                max = 0
                for OBJ in objs:
                    bbox = OBJ.bbox.scale(scale_x, scale_y)
                    area = (int(bbox.xmax) - int(bbox.xmin))* (int(bbox.ymax) - int(bbox.ymin))
                    if area > max:
                        curr_obj = OBJ
                        print(f'there are multiple')
        else:
            curr_obj is None
        if curr_obj is not None:
            #print(curr_obj.score)
            if(curr_obj.score > 0.55): 
                    Confirmed_sign = curr_obj 
                    sign_class = Confirmed_sign.id 
                    bbox = curr_obj.bbox.scale(scale_x, scale_y)
                    sign_bounds[0] = int(bbox.xmin), int(bbox.ymin)
                    sign_bounds[1] = int(bbox.xmax), int(bbox.ymax)
                    sign_center = (int(bbox.xmin) + int(bbox.xmax))//2
                    print(f'SIGN BOUNDS{sign_bounds}')
                    count = 0
            else: count = 0

# [FUNCTION] The start function is run once every time the start button is pressed
def remap_range(value, old_lower, old_upper, new_lower, new_upper):
    normalized = (value - old_lower) / (old_upper - old_lower)
    new_value = normalized * (new_upper - new_lower) + new_lower
    return new_value

def start():
    global speed
    global angle
    global start_time

    start_time = time.time()

    startCSVfile()
# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  


def update():
    
    global sign_class 
    global sign_bounds 
    global Confirmed_sign
    global sign_center

    global last_offset
    global integral_error
    global angle 
    global speed 
    global CurrFunc
    global left_avg
    global right_avg
    

    # Search for signs 
    update_object()
    print(Confirmed_sign is None)
    #angle control
    if (Confirmed_sign is not None):
        setpoint = rc.camera.get_width() //2 # middle - 0
        offset = sign_center - setpoint 
        print(sign_center , "and", offset)
        kp = 0.005
        kd = 0.000
        speed = 0.7
        P = kp * (offset) 
        D = kd * (offset - last_offset)
        angle = P +D 
        last_offset = offset
    else: 
        speed = 0
        angle = 0
    angle = rc_utils.clamp(angle, -1, 1)

  #  CSVMAKE(True)

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
    print(f'Angle {angle} and Speed {speed}')
    print(sign_center)
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