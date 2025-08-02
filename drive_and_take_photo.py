# this code does manual drive and click right trigger to take photos
########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
from PIL import Image
import os
import cv2

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle
global speed_offset
global angle_offset

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global speed_offset
    global angle_offset

    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle away from the center is at 0.0

    # This tells the car to begin at a standstill
    rc.drive.stop()
diff = 0
def update():
    global speed
    global angle
    global speed_offset
    global angle_offset
    global diff

    # speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # if abs(speed) < 0.1:
    #     speed = 0
    
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    (x2, y2) = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    angle = x2
    speed = y
    diff -= rc.get_delta_time()
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) and diff < 0:
        print("Taking Foto")
        img = rc.camera.get_color_image()
        rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)
        num_files = len(os.listdir("images/"))
        pil_img.save(f"images/photo_new_sign_{num_files}.jpg")
        diff = 0.5

    # Send the speed and angle values to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
