"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: grand_prix.py

Title: Grand Prix Day!

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR will traverse the obstacle course autonomously without human intervention.
Once the start button is pressed, the RACECAR must drive through the course until it
reaches finish line.

Note: There is no template code in this document to follow except for the RACECAR script 
structure found in template.py. You are expected to use code written from previous labs
to complete this challenge. Good luck!

Expected Outcome: When the user runs the script, they must not be able to manually control
the RACECAR. The RACECAR must move forward on its own, traverse through the course, and then
stop on its own.
"""

##NOTE THAT ACTUAL VALUES ARE IN THE GITHUB ONE

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(2, '../../library')
import racecar_core
import racecar_utils as rc_utils
import math
import cv2 as cv
import numpy as np
import os

from conesla import ConeAvoider
from wallfollow import WallFollower
from dynamicOBclass import DynamicObstacle


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Load dictionary and parameters from the aruco library
arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
arucoParams = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(arucoDict, arucoParams)

#GLOBAL VARIABLES
speed = 0.0
angle = 0.0
#STATE SWITCHING
STATES = {5: "WALLSHEN", 0: "WALLFOLLOW", 1: "WALLSHEN" , 2: "CONE", 3: "DYNAM OBSTACLE", 4:"LINEFOLLOW/WALL"}
current_state = STATES[5]
markertimer = 0.0
markercount = 0
#WALL INPUTS
left_avg =0.0
right_avg = 0.0
#LINE FOLLOW 1 INPUTS
MIN_CONTOUR_AREA = 30
left_center_offset = 0.0 
right_center_offset = 0.0




########################################################################################
# Functions
########################################################################################

#DATA COLLECTION FUNCTIONS -----------------------------------
# def update_lidar():
#     global left_avg
#     global right_avg

#     scan = rc.lidar.get_samples()
#     left_avg = rc_utils.get_lidar_average_distance(scan,300,60)
#     right_avg =rc_utils.get_lidar_average_distance(scan,60,60)
#     if left_avg > 500 :
#         left_avg = rc_utils.get_lidar_average_distance(scan,350,10)
#         if left_avg > 300 : left_avg = 0
#     if right_avg > 500 : 
#         right_avg = rc_utils.get_lidar_average_distance(scan,10,10)
#         if right_avg > 300 : right_avg = 0

def updateStateAndMarker():
    global markercount 
    global current_state
    global STATES
    global markertimer 

    image = rc.camera.get_color_image()
    corners, ids, _ = detector.detectMarkers(image)
    for x in range(len(corners)): # will not run if no corners are detected!
        curr_corners = corners[x][0]
        area = abs((curr_corners[2][0] - curr_corners[0][0])) * abs((curr_corners[2][1] - curr_corners[0][1]))
      #  print(area)
        if area > 1200:
            current_state = STATES[ids[0]] #only would work if theres 1 AR marker...
    markertimer += rc.get_delta_time()



#DRIVING FUNCTIONS
# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global anglewall
    global speed 
    global angle 
    global lines1anglePID
    global current_state
    global wf
    global ca
    global db 

    ca = ConeAvoider(rc)
    wf = WallFollower(rc) 
    db = DynamicObstacle(rc)

    db.start()
    current_state = STATES[5]

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global angle 
    global speed 
    global anglewall
    global lines1anglePID
    global left_center_offsetls
    global right_center_offset
    global speed, angle
    global error, ca, wf

    updateStateAndMarker()

    if current_state == "WALLSHEN":
        speed, angle = wf.update()
    if current_state == "WALLFOLLOW":
        # update_contour_line1()
        # speed = 0.7
        # error = abs(left_center_offset - right_center_offset)
        # angle = lines1anglePID.update(error)
        # angle = rc_utils.clamp(angle, -1, 1)
        # print(f'error: {error} and the angle: {angle}')
    if current_state == "CONE":
        speed, angle = ca.update()
    if current_state == "DYNAM OBSTACLE":
        speed, angle = db.update()
    if current_state == "LINEFOLLOW/WALL":
        if LF.detect():
            LF.update()
        else:
            wf.update()
        #if no line then do wallshen 

    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global current_state
    global markertimer
    global markercount

    print(f'state: {current_state}')

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
