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

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((rc.camera.get_height()//6, 0), (2*rc.camera.get_height()//3, rc.camera.get_width()))
# Color Contours
ORANGE = ((1, 170, 180), (15, 255, 255))
GREEN = ((36, 40, 89),  (68, 255, 255))

def cap(val):
    # clamp between -1 and 1
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
            for col in [ORANGE, GREEN]:
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
        
        self.speed = 0.52
        self.angle = 0
        if cntr is not None:
            # display on the car/print that a cone is seen
            if col == ORANGE:
                rc.display.show_text("Oran")
                print("ORANGE", area)  
            if col == GREEN:
                rc.display.show_text("gren")
                print("GREEN", area)  

            # set the "setpoint" to the cone position + or - an offset based on which cone it sees
            off = 1000
            setpt = self.rc.camera.get_width()//2 + (off if col == ORANGE else -off if col == GREEN else 0)
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


# ca stands for cone avoider, instance of the main class
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
 
def update():
    global speed, angle
    global error, ca

    speed, angle = ca.update()
    rc.drive.set_speed_angle(speed, angle)


def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()