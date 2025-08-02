import sys
import time
import math
import numpy as np
import cv2 as cv
from enum import Enum

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#global

# Line-following
contour_center = None
contour_area = 0
MIN_LINE_AREA = 300
CROP_FLOOR = ((300, 0), (400, rc.camera.get_width()))
LINE_COLORS = [((1, 110, 170), (27, 255, 253)),  # ORANGE
               ((56, 39, 171), (117, 255, 255))] # GREEN
               #((90, 55, 80), (120, 255, 255))] # BLUE


line_error_prev = 0.0
line_error_sum = 0.0
line_pid_KP = 1.5
line_pid_KI = 0.2
line_pid_KD = 0.01      
line_pid_last_time = time.time()
last_line_angle = 0.0
last_line_speed = 0.6


# ----------------------------------------------
# Utility
# ----------------------------------------------
def update_line_contour():
    global contour_center, contour_area
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
        return
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
    for color in LINE_COLORS:
        contours = rc_utils.find_contours(image, color[0], color[1])
        contour = rc_utils.get_largest_contour(contours, MIN_LINE_AREA)
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            return
    contour_center = None
    contour_area = 0

def a2ind(angle: float) -> int:
    return int((angle % 360) * 2)
# ----------------------------------------------
# Start
# ----------------------------------------------
def start():
    rc.drive.stop()
    rc.set_update_slow_time(1/5)
    print(">> Starting Integrated Autonomous Script")
    mode = None

# ----------------------------------------------
# Update
# ----------------------------------------------
def update():
    global line_error_prev, line_error_sum, line_pid_last_time
    global last_line_angle, last_line_speed

    scan = rc.lidar.get_samples()
    front_distances = [n for n in np.concatenate((scan[a2ind(-30):], scan[:a2ind(30)])) if n > 0]
    narrow_front_distances = [n for n in np.concatenate((scan[a2ind(-10):], scan[:a2ind(10)])) if n > 0]
    unblocked_total_front_distances = [n for n in np.concatenate((scan[a2ind(-90):], scan[:a2ind(90)])) if n > 300] # blocked distance def here (200)
    if front_distances:
        max_front_distance = np.partition(front_distances, -7)[-7] if len(front_distances) >= 7 else np.max(front_distances)
    if len(narrow_front_distances) <= 10:
        max_front_distance = 800

    
    update_line_contour()
    dt = max(time.time() - line_pid_last_time, 1e-3)

    if contour_center is not None:
        error = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        line_error_sum += error * dt
        deriv = (error - line_error_prev) / dt

        angle = (
            line_pid_KP * error +
            line_pid_KI * line_error_sum +
            line_pid_KD * deriv
        )
        if angle > 0.65:
            angle = 1
        elif angle < -0.65:
            angle = -1
        angle = np.clip(angle, -1.0, 1.0)
        if abs(angle) < 0.1:
            speed = 1.0
        elif abs(angle) < 0.3:
            speed = 0.9
        else:
            speed = 0.8
        if max_front_distance < 350:
            speed = 0.7

        line_error_prev = error
        line_pid_last_time = time.time()
        last_line_angle = angle
        last_line_speed = speed

    else:
        angle = last_line_angle
        speed = 0.7

    rc.drive.set_speed_angle(speed, angle)

# ----------------------------------------------
# Main
# ----------------------------------------------
if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()