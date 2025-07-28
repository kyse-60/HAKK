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
# import shapely as sp
import random
import heapq

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
        # ---- tunables ----
        self.amt_consider    = 120   # window size in samples (v)
        self.min_dist_thresh = 90  # minimum distance in window (m)
        self.avg_dist_thresh = 90    # average distance in window (m)
        self.cruise_speed    = 0.5    # forward speed
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