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
#STOP NEEDS STICK 
#GO AROUND MUST BE ON THE BLOCK


import sys
import cv2 as cv
import numpy as np
import os
import time
import math

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'signmodelupdate.tflite'
label_name = 'signs.txt'

model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 6

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, "./library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

def cap(val):
    return min(max(val, -1), 1)

class WallFollower():
    def __init__(self, rc):
        self.rc = rc
        # ---- tunables ----
        self.amt_consider    = 90   # window size in samples (v)
        self.min_dist_thresh = 90  # minimum distance in window (m)
        self.avg_dist_thresh = 90    # average distance in window (m)
        self.cruise_speed    = 0.8    # forward speed
        self.window_srch = 100
        # ------------------

    def update(self):
        scan = np.array(self.rc.lidar.get_samples())    # 720 samples @0.5° each
        # print(rc_utils.get_lidar_average_distance(scan, 0, 50))
        # print(scan[0:20])
        # scan[scan==0] = 500
        # print(len(scan))
        # print(scan[-180])#scan[180], scan[270*2])
        N    = len(scan)                                 # =720
        v    = self.amt_consider
        h_v  = v // 2

        # 1) build the ±45° region around front (0°):
        half_search = int(self.window_srch / 0.5)  # =90 samples each side
        # indices 630…719  wrap → 0…90
        left_idxs  = np.arange(N - half_search, N)
        right_idxs = np.arange(0, half_search + 1)
        region_idxs = np.concatenate((left_idxs, right_idxs))   # length 181

        # pull those distances into a linear array
        region = scan[region_idxs]

        best_score   = -np.inf
        best_center  = None

        # 2) slide your v‑wide window within the 181‑sample region
        for c in range(h_v, len(region) - h_v):
            window = region[c - h_v : c + h_v + 1]
            # only consider FULL‑SIZE windows
            if len(window) != v + 1:
                continue
            # check thresholds
            window = window[window != 0]
            if len(window) == 0:
                # ignore
                continue
            # print(window)
            if window.min() > self.min_dist_thresh and window.mean() > self.avg_dist_thresh:
                m = window.max()
                if m > best_score:
                    # print(m, c)
                    best_score  = m
                    best_center = c

        # 3) if we found a valid corridor, map it back to steering
        if best_center is not None:
            scan_idx = region_idxs[best_center]

            # convert scan_idx to relative angle in degrees:
            #   scan_idx*0.5 gives [0…360)°; anything >45 must be on the right side,
            #   so subtract 360 to wrap into [−45…+45].
            angle_deg = (scan_idx * 0.5)
            if angle_deg > self.window_srch:
                print(angle_deg)
                angle_deg -= len(scan)/2

            # normalize ±45° → ±1
            # if 2*angle_deg / self.window_srch > 1:
            #     speed = 0.5 * self.cruise_speed
            # else:
            #     speed    = self.cruise_speed
            speed    = self.cruise_speed
            steering = cap(angle_deg / (self.window_srch))
            print(angle_deg, best_score)
            rc.display.show_text("A")
        else:
            # no valid gap found → stop
            speed    = 0.5
            steering = 0.0

        return speed, steering
        return 0, 0


# >> Constants
# The smallest contour we will recognize as a valid contour
CROP = ((rc.camera.get_height()//3, 0), (rc.camera.get_height(), rc.camera.get_width()))

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
interpreter = None
wf = None
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
    global interpreter
    
    image = rc.camera.get_color_image()
  #  image = rc_utils.crop(image, CROP[0], CROP[1])

    if image is None:
        current_sign = None
        sign_bounds = [0,0]
    else:    
        #Loading model and labels
        # interpreter.allocate_tensors()
        # labels = read_label_file(label_path)
        # inference_size = input_size(interpreter)
        #running model over image and getting objs 
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()
        labels = read_label_file(label_path)
        inference_size = input_size(interpreter)
        scale_x, scale_y = rc.camera.get_width() / inference_size[0], rc.camera.get_height() / inference_size[1]
        print(inference_size)
        image = cv.resize(image, inference_size)
        run_inference(interpreter, image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        if(len(objs) > 0):
            print("it's detecting")
            if(len(objs)== 1):
                curr_sign_obj = objs[0]
            else:
                print("BAD!-COME BACK TO")
        else: curr_sign_obj = None
        if curr_sign_obj is not None:
            print(f'score: {curr_sign_obj.score} and class: {curr_sign_obj.id}')
            if(curr_sign_obj.score > 0.1): 
                #sign_class = curr_sign_obj.id
            # if(sign_class == last_class):
            #     count += 1
            #     if count >= 1: 
                Confirmed_sign = curr_sign_obj 
                sign_class = Confirmed_sign.id 
                bbox = curr_sign_obj.bbox.scale(scale_x, scale_y)
                sign_bounds[0] = int(curr_sign_obj.bbox.xmin), int(curr_sign_obj.bbox.ymin)
                sign_bounds[1] = int(curr_sign_obj.bbox.xmax), int(curr_sign_obj.bbox.ymax)
                sign_center = (int(bbox.xmin) + int(bbox.xmax))//2
                print(f'SIGN BOUNDS{sign_bounds}')
                count = 0
            # else: count = 0
            last_class = sign_class 
        else: Confirmed_sign = None
        

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
    global interpreter
    global wf
    wf = WallFollower(rc)
    # interpreter = make_interpreter(model_path)
    # time.sleep(2)
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
    global wf
    

    # Search for signs 
    #update_lidar()

    # #angle control
    scan = rc.lidar.get_samples()
    left_dist = rc_utils.get_lidar_average_distance(scan, 3*len(scan)/4, 20)
    right_dist = rc_utils.get_lidar_average_distance(scan, len(scan)/4, 20)
    
    if (Confirmed_sign is not None and sign_class == 4):
        if right_dist > left_dist:
            angle = -0.15
        else:
            angle = 0.15
        # offset = rc.camera.get_width() //2 # middle - 0
        # kp = 0.005
        # P = kp * (offset)
    else: 
        # #else do line follower 
        # offset = right_avg - left_avg
        # offset_norm = offset / OFFSET_FULL_STEER
        # P = 1 * math.tanh(GAIN *offset_norm)
        # # kp = 0.0062
        # kd = 0.12
        # ki = 0.000#1
        # integral_error += (rc.get_delta_time() * offset)
        # #   P = kp * (offset)
        # I = ki * integral_error
        # D = kd * (offset - last_offset)
        # last_offset = offset
        # angle = P + I + D #+ CurrFunc(offset)
        _, angle = wf.update()

    angle = rc_utils.clamp(angle, -1, 1)

    #speed control
    if Confirmed_sign is not None:
        sign_class = Confirmed_sign.id
        if(sign_class == 0):
            print("ran into stop")
            speed = -0.2 #maybe make negative needs testing
        if(sign_class == 2):
            print("yield")
            speed = 0.3
    else:
       # print("normal speed")
        speed = remap_range(speed, 0,1, 0.6, 0.8) # in wall follow this is speed = 1 but thats dangerous


    # print(speed)
    # print(angle)
    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global angle
    global speed
    global sign_class 
    global sign_bounds 
    global Confirmed_sign
    global sign_center
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    print(f'Angle {angle} and Speed {speed}')
    update_object()
   # update_object()
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