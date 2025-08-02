"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-summer-labs

File Name: lab_2a.py

Title: BYOA (Build Your Own AHRS)

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: The goal of this lab is to build and deploy a ROS node that can ingest
IMU data and return accurate attitude estimates (roll, pitch, yaw) that can then
be used for autonomous navigation. It is recommended to review the equations of
motion and axes directions for the RACECAR Neo platform before starting. Template
code has been provided for the implementation of a Complementary Filter.

Expected Outcome: Subscribe to the /imu and /mag topics, and publish to the /attitude
topic with accurate attitude estimations.
"""

import rclpy
import racecar_core
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import numpy as np
import math
import time

rc = racecar_core.create_racecar()

class CompFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # Set up subscriber and publisher nodes
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.publisher_velocity = self.create_publisher(Vector3, '/velocity', 10) # output as [roll, pitch, yaw] angles

        self.prev_time = self.get_clock().now() # initialize time checkpoint
        self.alpha = 0.98 # TODO: Determine an alpha value that works with the complementary filter

        # set up attitude params
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    # [FUNCTION] Called when new IMU data is received, attidude calc completed here as well
    def imu_callback(self, data):
        # TODO: Grab linear acceleration and angular velocity values from subscribed data points
        accel = data.linear_acceleration
        # TODO: Calculate time delta
        now = rclpy.Time.now() # Current ROS time
        dt = now - self.prev_time

        self.vx = self.vx + accel[0]*dt
        self.vy = self.vy + accel[1]*dt
        self.vz = self.vz + accel[2]*dt
        # Print results for sanity checking
        print(f"====== Complementary Filter Results ======")
        print(f"Speed || Freq = {round(1/dt,0)} || dt (ms) = {round(dt*1e3, 2)}")
        print("\n")
        velocity = Vector3()
        velocity.x = self.vx
        velocity.y = self.vy
        velocity.z = self.vz
        self.publisher_velocity.publish(velocity)
        # TODO: Publish to velocity topic
        
    
    
def main():
    rclpy.init(args=None)
    node = CompFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()