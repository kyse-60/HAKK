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

def fit_circle_ls(x, y):
    """
    Fit a circle to >3 points (xi, yi) via linear least squares.
    Returns (xc, yc, R).
    """
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x*x + y*y)
    D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
    xc, yc = -D/2, -E/2
    R = np.sqrt(xc*xc + yc*yc - F)
    return xc, yc, R

def fill_missing_lidar_circular(scan, angles, window=5):
    N   = scan.size
    out = scan.astype(float).copy()
    zero = (out == 0)

    # 1) find all zero–spans linearly
    spans = []
    in_zero = False
    for i, z in enumerate(zero):
        if z and not in_zero:
            start, in_zero = i, True
        elif (not z) and in_zero:
            spans.append((start, i-1)); in_zero = False
    if in_zero:
        spans.append((start, N-1))

    # 2) only merge if we actually have at least two spans,
    #    and the first starts at 0 and the last ends at N-1
    if len(spans) >= 2 and spans[0][0] == 0 and spans[-1][1] == N-1:
        s0, e0 = spans.pop(0)
        s1, e1 = spans.pop(-1)
        spans.insert(0, (s1, e0))

    # 3) if spans is empty, there’s nothing to fill
    if not spans:
        return out

    # 4) process each span exactly as before
    for s, e in spans:
        # collect neighbor indices mod N
        left_idxs  = [(s + i) % N for i in range(-window, 0)]
        right_idxs = [(e + i) % N for i in range(1, window+1)]
        neigh = np.array(left_idxs + right_idxs, dtype=int)
        valid = neigh[out[neigh] > 0]
        if valid.size < 3:
            continue

        # build XY coords
        rs     = out[valid]
        thetas = angles[valid]
        xs, ys = rs*np.cos(thetas), rs*np.sin(thetas)

        # fit circle (least squares)
        xc, yc, R = fit_circle_ls(xs, ys)

        # interpolation anchors (wrapped)
        li = (s-1) % N
        ri = (e+1) % N
        # linearized positions for interpolation
        li_m = li if li >= s else li + N
        ri_m = ri if ri >= s else ri + N

        # figure out missing indices (handles wrap)
        if s <= e:
            missing = range(s, e+1)
        else:
            missing = list(range(s, N)) + list(range(0, e+1))

        # fill each missing beam
        for k in missing:
            a = angles[k]
            C = xc*np.cos(a) + yc*np.sin(a)
            disc = C*C - (xc*xc + yc*yc - R*R)

            # “linear” fallback
            k_m = k if k >= s else k + N
            lin = np.interp(k_m, [li_m, ri_m], [out[li], out[ri]])

            if disc < 0:
                r_est = lin
            else:
                r1 = C + np.sqrt(disc)
                r2 = C - np.sqrt(disc)
                r_est = r1 if abs(r1-lin) < abs(r2-lin) else r2

            out[k] = r_est

    return out
def remap_range(value, old_lower, old_upper, new_lower, new_upper):
    normalized = (value - old_lower) / (old_upper - old_lower)
    new_value = normalized * (new_upper - new_lower) + new_lower
    return new_value

class WallFollower():
    def __init__(self, rc):
        self.rc = rc
        # ---- tunables ----
        # parameters for low speed
        self.amt_consider    = 90   # window size in samples (v)
        self.min_dist_thresh = 90  # minimum distance in window (m)
        self.avg_dist_thresh = 90    # average distance in window (m)
        self.cruise_speed    = 0.7    # forward speed
        self.window_srch = 90
        # ------------------
        # parameters for high speed
        # self.amt_consider    = 160   # window size in samples (v)
        # self.min_dist_thresh = 100  # minimum distance in window (m)
        # self.avg_dist_thresh = 100    # average distance in window (m)
        # self.cruise_speed    = 0.9    # forward speed
        # self.window_srch = 100
         # ------------------
        # parameters for cornering
        # self.amt_consider    = 160   # window size in samples (v)
        # self.min_dist_thresh = 50  # minimum distance in window (m)
        # self.avg_dist_thresh = 50    # average distance in window (m)
        # self.cruise_speed    = 0.6    # forward speed
        # self.window_srch = 180
        # ------------------

    def update(self):
        scan = np.array(self.rc.lidar.get_samples())    # 720 samples @0.5° each
        # print(rc_utils.get_lidar_average_distance(scan, 0, 50))
        # print(scan[0:20])
        # scan[scan==0] = 500
        # print(len(scan))
        # print(scan[-180])#scan[180], scan[270*2])
        # if len(scan) == 0:
        #     return 0.5, 0
        # scan[scan > 300] = 0
        # poses = scan==0
        # scan = fill_missing_lidar_circular(scan, np.linspace(-np.pi, np.pi, scan.size, endpoint=False))
        # scan[poses] += 50
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
            # speed    = self.cruise_speed
            angle = cap(angle_deg / (self.window_srch))
            # angle = cap(4*angle_deg / (self.window_srch))
            if angle < -0.4:
                speed = remap_range(angle, -1, 0, 0.6, 0.8)
            elif angle > 0.4:
                speed = remap_range(angle, 0, 1, 0.8, 0.6)
            else: 
                speed = 0.8
            print(angle_deg, best_score)
            rc.display.show_text(str(int(angle_deg)))
        else:
            # no valid gap found → stop
            speed    = 0.5
            angle = 0.0

        return speed, angle



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