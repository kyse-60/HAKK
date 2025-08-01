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
# MIN_CONTOUR_AREA = 50

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((rc.camera.get_height()//6, 0), (2*rc.camera.get_height()//3, rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
# BLUE = ((91, 150, 46), (122, 255, 255))  # The HSV range for the color blue
# BLUE = ((90, 150, 50), (120, 255, 255))  # The HSV range for the color blue
# GREEN = ((35, 50, 50), (75, 255, 255))  # The HSV range for the color green
# RED = ((0, 50, 50), (10, 255, 255)) # The HSV range for the color red
# RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
# ORANGE = ((9, 54, 169), (90, 255, 255)) # The HSV range for the color red
ORANGE = ((1, 170, 180), (15, 255, 255)) # The HSV range for the color red
# ORANGE = ((9, 68, 169), (15, 255, 255)) # The HSV range for the color red
GREEN = ((36, 40, 89),  (68, 255, 255))
YELLOW = ((20, 146, 46), (44, 255, 255))
# Color priority: Red >> Green >> Blue
# COLOR_PRIORITY = (RED, BLUE)
RED = ORANGE
BLUE = GREEN

def cap(val):
    return min(max(val, -1), 1)

class ConeAvoider():
    def __init__(self, rc):
        self.rc = rc
        self.MIN_CONTOUR_AREA = 700
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
            contr = None
            # image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
            for col in [RED, BLUE]:
                contours = rc_utils.find_contours(image, col[0], col[1])
                contour = rc_utils.get_largest_contour(contours, self.MIN_CONTOUR_AREA)
                if contour is not None:
                    self.contour_center, self.contour_area = (rc_utils.get_contour_center(contour), rc_utils.get_contour_area(contour))
                    # if mx_area:
                    #     return None, (self.contour_area+mx_area)/2, ((self.contour_center[0]+cntr[0])/2, (self.contour_center[1]+cntr[1])/2)
                    if self.contour_area > mx_area:
                        mx_area = self.contour_area
                        cntr = self.contour_center
                        clr = col
                        contr = contour
            # if contr is not None:
            #     rc_utils.draw_contour(image, contr, rc_utils.ColorBGR.brown.value)
            #     self.rc.display.show_color_image(image)
            return clr, mx_area, cntr
    
    def update(self):
        # print("updating")
        self.angle = 0


        # Search for contours in the current color image
        col, area, cntr = self.update_contour()
        

        # if len(self.prev_poses) > 5:
        #     self.prev_poses.pop(0)
        #     self.prev_amts.pop(0)
        # scan = np.copy(self.rc.lidar.get_samples())
        # scan[180:360+180] = 10000
        # if col == RED:
        #     self.prev_cl = RED
        # if col == BLUE:
        #     self.prev_cl = BLUE

        # if self.prev_cl == RED:
        #     scan[:180] = 10000
        # if self.prev_cl == BLUE:
        #     scan[360+180:] = 10000
        # scan[scan == 0] = 10000
        # pos = np.argmin(scan)
        # amt = scan[pos]-20
        # self.prev_poses.append(pos)
        # self.prev_amts.append(amt)
        self.speed = 0.52
        self.angle = 0
        if cntr is not None:
            # print("oioioi")
            if col == RED:
                rc.display.show_text("Oran")
                print("ORANGE", area)  
            if col == BLUE:
                rc.display.show_text("gren")
                print("GREEN", area)  
            # rc.display.show_text("Cone")
            off = 1000
            setpt = self.rc.camera.get_width()//2 + (off if col == RED else -off if col == BLUE else 0)
            line = cntr[1] 
            # Kp = 20/amt
            Kp = 0.01
            err = (setpt-line)
            self.angle = 0
            if area > 3000 and (self.rc.camera.get_width()//2 - line) < 300 and area < 18000:
                self.angle = cap(err*Kp)
                self.cmds.append(-self.angle)
                self.cnt += 1
                if self.cnt > 4:
                    self.not_seen = 0
                    if self.angle:
                        self.prev_angle = self.angle
                self.cnt2 = 0
        else:
            rc.display.show_text("")
            self.cnt = 0
            self.cnt2 += 1
            self.not_seen += 1
            # avg_pos = np.median(self.prev_poses)
            # avg_amt = np.median(self.prev_amts)
            # print(avg_pos, avg_amt, self.prev_angle, self.not_seen)
            self.angle = 0
            if self.not_seen > 2 and self.not_seen < 150:
                # if avg_amt < 30:
                #     self.angle = 1 if avg_pos < 360 else -1
                # else:
                self.angle = -1 if self.prev_angle > 0 else 1 if self.prev_angle < 0 else 0
                # if len(self.cmds):
                #     self.angle = self.cmds.pop(-1)
                # else:
                #     self.angle = 0
                # print("ayyy", self.angle, self.not_seen)
        # if abs(self.angle) == 1:
        #     self.speed = 0.7
        print(self.speed, self.angle)
        # if amt < 30:
        #     self.angle = 0
        # self.angle /= 2
        # rc.display.show_text("hello")
        return self.speed, self.angle


ca = None

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global ca
    ca = ConeAvoider(rc)
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0, 0)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed, angle
    global error, ca

    speed, angle = ca.update()
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