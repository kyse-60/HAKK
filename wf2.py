"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: wf2.py

Title: Wall Follower 2

Author: RACECAR Team 7

Purpose: Wall Following on the 30 meter long course

Expected Outcome: Successfully navigates the track!
"""

########################################################################################
# Importstfvtfvvfvtfv
########################################################################################

import sys
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
import numpy as np
from scipy.ndimage import convolve1d


# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global contour_center, contour_area
global speed, angle
global error

def cap(val):
    return min(max(val, -1), 1)

class WallFollower():
    def __init__(self, rc):
        self.rc = rc
        self.cnt = 0
        self.h = 0
        self.prev_diff = 0
        self.prev_sent = (1, 0)
        self.l_frame = []
        self.r_frame = []
        self.f_frame = []
    def update(self):
        # get left, right, and forward scan data, 
        # using the median of the past 5 measurements for smoothing
        scan = np.copy(self.rc.lidar.get_samples())
        self.r_frame.append(rc_utils.get_lidar_average_distance(scan, 90-45, 10))
        self.l_frame.append(rc_utils.get_lidar_average_distance(scan, 270+45, 10))
        self.f_frame.append(rc_utils.get_lidar_average_distance(scan, 0, 10))
        if len(self.r_frame) > 5:
            self.r_frame.pop(0)
            self.l_frame.pop(0)
            self.f_frame.pop(0)
        right = np.median(self.r_frame)
        left = np.median(self.l_frame)
        fwd = np.median(self.f_frame)
        # if a bad measurement, just do what we did last
        if right == 0 or left == 0 or fwd == 0:
            return self.prev_sent
        # using a proportion, if the walls are wider this code should still work (not hard coded for track width)
        diff_prop = (right-left)/(right+left)
        Kp = 2
        Kd = 4
        # one time unit d-term
        d_term = (diff_prop - self.prev_diff)/ rc.get_delta_time()
        angle = Kp*diff_prop + Kd*d_term
        # a better speed control may be better
        # may need to change for actual car
        speed = 1 if fwd > 50 else fwd/100

        self.prev_diff = diff_prop
        self.prev_sent = cap(speed), cap(angle)
        return self.prev_sent # this is now a bit misleading, prev_sent is in this moment the current speed and angle

wf = None

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global wf
    wf = WallFollower(rc)
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0, 0)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed, angle
    global error, wf

    speed, angle = wf.update()
    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()