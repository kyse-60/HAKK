"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: grand_prix.py

Title: Wall Follower 2

Author: RACECAR Team 7

Purpose: Grand Prix

Expected Outcome: Does not DNF!
"""

########################################################################################
# Imports
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
# Global variables/util function
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

class LineFollows():
    def __init__(self, rc):
        # The smallest contour we will recognize as a valid contour
        self.MIN_CONTOUR_AREA = 50

        # A crop window for the floor directly in front of the car
        self.CROP_FLOOR = ((rc.camera.get_height()//6, 0), (4*rc.camera.get_height()//6, rc.camera.get_width()))

        # TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
        # Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
        self.BLUE = ((90, 150, 50), (120, 255, 255))  # The HSV range for the color blue
        # GREEN = ((35, 50, 50), (75, 255, 255))  # The HSV range for the color green
        self.RED = ((0, 50, 50), (10, 255, 255)) # The HSV range for the color red
        # ORANGE = ((9, 54, 169), (90, 255, 255)) # The HSV range for the color red
        self.ORANGE = ((4, 75, 169), (15, 255, 255)) # The HSV range for the color red
        # ORANGE = ((9, 68, 169), (15, 255, 255)) # The HSV range for the color red
        self.GREEN = ((30, 58, 57), (80, 255, 255))
        # Color priority: Red >> Green >> Blue
        self.COLOR_PRIORITY = (self.BLUE, self.GREEN) # swap order for dynamic obstacle
        self.rc = rc


        # >> Variables
        self.speed = 0.0  # The current speed of the car
        self.angle = 0.0  # The current angle of the car's wheels
        self.contour_center = None  # The (pixel row, pixel column) of contour
        self.contour_area = 0  # The area of contour
        self.cntr = 0
        self.last_error = 0
        self.last_angle  = 0
        self.past_five = []
        self.counter = 0
    def update_contour(self):

        image = self.rc.camera.get_color_image()

        if image is None:
            self.contour_center = None
            self.contour_area = 0
        else:
            # Crop the image to the floor directly in front of the car
            image = rc_utils.crop(image, self.CROP_FLOOR[0], self.CROP_FLOOR[1])

            # TODO Part 2: Search for line colors, and update the global variables
            # contour_center and contour_area with the largest contour found
            for color in self.COLOR_PRIORITY:
                contours = rc_utils.find_contours(image, color[0], color[1])

                contour = rc_utils.get_largest_contour(contours, self.MIN_CONTOUR_AREA)

                if contour is not None:
                        self.contour_center = rc_utils.get_contour_center(contour)
                        self.contour_area = rc_utils.get_contour_area(contour)

                        rc_utils.draw_contour(image,contour)
                        rc_utils.draw_circle(image, self.contour_center)
                        break 


    def update(self):
        """
        After start() is run, this function is run every frame until the back button
        is pressed
        """
        # Search for contours in the current color image
        self.update_contour()


        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle

        if self.contour_center is not None:
            setpoint = rc.camera.get_width()//2
            error = (setpoint - self.contour_center[1])
            kp = -0.0045   
            kd = -0.005 #-0.001, -0.0007
            # kp = -0.0055    #-0.06 osolates like crazy, not in a bangbangy kinda way but not staying on the line                     #0.05 works but dive #0.007
            # kd = -0.0004#-0.0065 # -0.006, -0.009
            self.past_five.append(error)
            dterm = 0
            if len(self.past_five) > 5:
                self.past_five.pop(0)
            if len(self.past_five) == 5:
                e0, e1, e2, e3, e4 = self.past_five
                # apply the five-point backward-difference formula
                dterm = (25*e4- 48*e3+ 36*e2- 16*e1+  3*e0) / (12)
            self.angle = kp * error + kd*dterm
            self.angle = max(-1, min(1, self.angle))
            self.cntr = 0

            print("speed: ", round(self.speed, 2), " angle: ", round(self.angle, 2), " lastangle: ", round(self.last_angle, 2), " error: " , round(error, 2), " dterm :", round(dterm, 2), " contour_area: ", round(self.contour_area, 2), " contour_center: ", self.contour_center)


            
            if(self.last_angle == self.angle):
                self.counter += 1
                if(self.counter >= 10):
                    print("===============================")
                    print("contour_area: ", round(self.contour_area, 2), " contour_center: ", self.contour_center)
                    print("===============================")
            else:
                self.counter = 0

            
            self.last_error = error 
            self.last_angle = self.angle 
            
        else: 
            print("dont see anything")
            cntr += 1
            if cntr > 5:
                self.angle = self.last_angle 

        if angle < -0.2:
            self.speed = remap_range(self.angle, -1, 0, 0.6, 0.9)
        elif angle > 0.2:
            self.speed = remap_range(self.angle, 0, 1, 0.9, 0.6)
        else: 
            self.speed = 0.6

        return self.speed, self.angle

class WallFollower():
    def __init__(self, rc, amt_consider=90, min_dist_thresh=90, avg_dist_thresh=90, cruise_speed=1, window_srch=90, angle_mul=3):
        self.rc = rc
        # ---- tunables ----
        self.amt_consider    = amt_consider   # window size in samples (v)
        self.min_dist_thresh = min_dist_thresh  # minimum distance in window (m)
        self.avg_dist_thresh = avg_dist_thresh    # average distance in window (m)
        self.cruise_speed    = cruise_speed    # forward speed
        self.window_srch = window_srch
        self.angle_mul = angle_mul
        # ------------------

    def update(self):
        scan = np.array(self.rc.lidar.get_samples())    # 720 samples @0.5° each
        if len(scan) == 0:
            return 0.5, 0
        scan[scan > 300] = 0
        poses = scan > 300
        scan = fill_missing_lidar_circular(scan, np.linspace(-np.pi, np.pi, scan.size, endpoint=False))
        # scan[poses] += 50
        # rc.display.show_lidar(scan)
        fwd = rc_utils.get_lidar_average_distance(scan, 0, 10)
        # scan = fill_missing_lidar_circular(scan, np.arange(len(scan), dtype=float)*2*np.pi/len(scan))
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
                # print(angle_deg)
                angle_deg -= len(scan)/2

            # normalize ±45° → ±1
            # if 2*angle_deg / self.window_srch > 1:
            #     speed = 0.5 * self.cruise_speed
            # else:
            #     speed    = self.cruise_speed

            speed    = self.cruise_speed * min(1, fwd/100)
            steering = cap(self.angle_mul*angle_deg / (self.window_srch))
            # print(angle_deg, best_score)
            # rc.display.show_text("A")
        else:
            # no valid gap found → stop
            speed    = 0.5
            steering = 0.0

        return speed, steering

class WallFollowerRamp():
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
        self.r_frame.append(rc_utils.get_lidar_average_distance(scan, 90-45, 30))
        self.l_frame.append(rc_utils.get_lidar_average_distance(scan, 270+45, 30))
        self.f_frame.append(rc_utils.get_lidar_average_distance(scan, 0, 10))
        if len(self.r_frame) > 5:
            self.r_frame.pop(0)
            self.l_frame.pop(0)
            self.f_frame.pop(0)
        right = np.median(self.r_frame)
        left = np.median(self.l_frame)
        fwd = np.median(self.f_frame)
        # if a bad measurement, just do what we did last
        if right == 0 or left == 0:
            return self.prev_sent
        # using a proportion, if the walls are wider this code should still work (not hard coded for track width)
        diff_prop = (right-left)/(right+left)
        Kp = 0.4
        Kd = 5
        # one time unit d-term
        d_term = (diff_prop - self.prev_diff)/ rc.get_delta_time()
        angle = Kp*diff_prop #+ Kd*d_term
        # a better speed control may be better
        # may need to change for actual car
        # speed = 1 if fwd > 50 else fwd/100
        speed = 1
        # print("ramp", speed, angle, right)
        self.prev_diff = diff_prop
        self.prev_sent = cap(speed), cap(angle)
        return self.prev_sent # this is now a bit misleading, prev_sent is in this moment the current speed and angle

class PablosWallFollower():
    def __init__(self, rc):
        self.rc = rc

    def update(self):
        RAY_FROM_OFFSET = 50.0
        WINDOW = 3.0
        kp = 0.0042
        speed = 1

        scan = self.rc.lidar.get_samples()

        right_abs_angle, right_abs_distance = rc_utils.get_lidar_closest_point(scan, (0, 180))
        left_abs_angle, left_abs_distance = rc_utils.get_lidar_closest_point(scan, (181, 360))
        right_offset_distance = rc_utils.get_lidar_average_distance(scan, RAY_FROM_OFFSET, WINDOW)
        left_offset_distance = rc_utils.get_lidar_average_distance(scan, (360 - RAY_FROM_OFFSET), WINDOW)
        front_distance = rc_utils.get_lidar_average_distance(scan, 0, 3)

        if front_distance < 340:
            speed = 0.75
            kp = 0.0048 # 0.0044
            print("uh-oh")
        else:
            speed = 0.85
            kp = 0.003

        right_angle = right_abs_angle - RAY_FROM_OFFSET
        left_angle = (360 - RAY_FROM_OFFSET) - left_abs_angle

        right_math = (right_offset_distance ** 2) - (right_abs_distance ** 2)
        left_math = (left_offset_distance ** 2) - (left_abs_distance ** 2)

        if right_math < 0:
            right_math = 0
        if left_math < 0:
            left_math = 0
        
        right_parallel_distance = math.sqrt(right_math)
        left_parallel_distance = math.sqrt(left_math)

        error = right_parallel_distance - left_parallel_distance

        angle = error * kp

        if(angle > 1):
            angle = 1
        elif(angle < -1):
            angle = -1

        self.rc.drive.set_speed_angle(speed, angle)
        print(error)

class ConeAvoider():
    def __init__(self, rc):
        self.rc = rc
        self.MIN_CONTOUR_AREA = 700
        self.ORANGE = ((1, 170, 180), (15, 255, 255))
        self.GREEN = ((36, 40, 89),  (68, 255, 255))
        self.PREV_COL = ""
        self.reverse_start = 0
        self.cooldown = False
        self.redcnt = 0
        self.bluent = 0
        self.prev_angle = 0
        self.not_seen = 0
        self.cnt = 0
        self.cnt2 = 0
        self.prev_poses = []
        self.prev_amts = []

        self.speed = 0
        self.angle = 0
        self.contour_center = None
        self.contour_area = 0
        self.prev_cl = None
        self.cmds = []
    
    def update_contour(self):
        image = self.rc.camera.get_color_image()
        if image is None:
            self.contour_center = None
            self.contour_area = 0
        else: 
            mx_area = 0
            clr = None
            cntr = None
            for col in [self.ORANGE, self.GREEN]:
                contours = rc_utils.find_contours(image, col[0], col[1])
                contour = rc_utils.get_largest_contour(contours, self.MIN_CONTOUR_AREA)
                if contour is not None:
                    self.contour_center, self.contour_area = (rc_utils.get_contour_center(contour), rc_utils.get_contour_area(contour))
                    if self.contour_area > mx_area:
                        mx_area = self.contour_area
                        cntr = self.contour_center
                        clr = col
            return clr, mx_area, cntr
    
    def update(self):
        self.angle = 0
        # Search for contours in the current color image
        col, area, cntr = self.update_contour()
        
        self.speed = 0.52 # reduce because speed upgrade gotten
        self.angle = 0
        if cntr is not None:
            # display on the car/print that a cone is seen
            if col == self.ORANGE:
                rc.display.show_text("Oran")
                print("ORANGE", area)  
            if col == self.GREEN:
                rc.display.show_text("gren")
                print("GREEN", area)  

            # set the "setpoint" to the cone position + or - an offset based on which cone it sees
            off = 1000
            setpt = self.rc.camera.get_width()//2 + (off if col == self.ORANGE else -off if col == self.GREEN else 0)
            line = cntr[1] 
            Kp = 0.01
            err = (setpt-line)

            self.angle = 0
            if area > 3000 and (self.rc.camera.get_width()//2 - line) < 300 and area < 18000:
                # if the cone contour size is in a certain range and we are close to the cone, then turn
                self.angle = cap(err*Kp)
                self.cmds.append(-self.angle)
                self.cnt += 1
                if self.cnt > 4:
                    # not just noise, consistent contour, so save this angle for un-turning after we turn
                    self.not_seen = 0
                    if self.angle:
                        self.prev_angle = self.angle
                self.cnt2 = 0
        else:
            rc.display.show_text("")
            self.cnt = 0
            self.cnt2 += 1
            self.not_seen += 1
            self.angle = 0
            if self.not_seen > 2 and self.not_seen < 150:
                # we haven't seen a cone for some time, so reverse turn 
                self.angle = -1 if self.prev_angle > 0 else 1 if self.prev_angle < 0 else 0

        print(self.speed, self.angle)
        return self.speed, self.angle
wf = None
wframp = None
wfturn = None
cur_driver = None
lf = None
ind = 0
states = None
tmr = 0


########################################################################################
# Functions
########################################################################################

def updt():
    global cur_driver
    if states[ind] == "wall":
        cur_driver = wf
    elif states[ind] == "ramp":
        cur_driver = wframp
    elif states[ind] == "right":
        cur_driver = wfright
    elif states[ind] == "turn":
        cur_driver = wfturn
    elif states[ind] == "line":
        cur_driver = lf

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global wf, wframp, ind, states, wfright, wfturn, lf
    ind = 0
    wf = WallFollower(rc, 120, 90, 90, 0.6, 90, 1)
    wframp = WallFollowerRamp(rc)
    wfturn = WallFollower(rc, 80, 90, 90, 0.5, 100, 3)
    lf = LineFollows(rc)
    # update below as needed
    states = ["wall", "wall", "wall", "cone", "line", "wall", "wall"]
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0, 0)
    updt()

def update():
    global speed, angle
    global error, states
    global tmr, cur_driver, ind, dt
    dt = rc.get_delta_time()
    image = rc.camera.get_color_image()
    # rc.display.show_color_image(image)
    
    # Find all AR Markers
    markers = rc_utils.get_ar_markers(image) # Markers object contains ID, corners, color, and color area params
    tmr -= dt
    speed, angle = cur_driver.update()
    # print("doing", speed, angle)
    rc.drive.set_speed_angle(speed, angle)
    if tmr > 0: return
    if len(markers):
        x = markers[0]
        corners = x.get_corners()
        area = abs((corners[2][0] - corners[0][0]) * (corners[2][1] - corners[0][1]))
        # print(area)
        if area > 300:
            tmr = 6
            ind += 1
            print("Changing operation to:", states[ind])
            updt()

def update_slow():
    pass 
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()