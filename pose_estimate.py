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
        self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.publisher_pose_estimate = self.create_publisher(Vector3, '/pose_estimate', 10) # output as [roll, pitch, yaw] angles

        self.prev_time = self.get_clock().now() # initialize time checkpoint
        self.alpha = 0.95 # TODO: Determine an alpha value that works with the complementary filter

        # set up attitude params
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.mag = None

    # [FUNCTION] Called when new IMU data is received, attidude calc completed here as well
    def imu_callback(self, data):
        # TODO: Grab linear acceleration and angular velocity values from subscribed data points
        accel = data.linear_acceleration
        gyro = data.angular_velocity

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
        accel_roll = math.atan2(accel.x,accel.y) # theta_x
        accel_pitch = math.atan2(-accel.x,(math.sqrt((accel.y**2) + (accel.z**2))))

        # TODO: Integrate gyroscope to get attitude angles
        gyro_roll = self.roll + gyro.x*dt # theta_xt
        gyro_pitch = self.pitch + gyro.y*dt # theta_yt
        gyro_yaw = self.yaw + gyro.z*dt # theta_zt

        # TODO: Compute yaw angle from magnetometer
        if self.mag:
            mx, my, mz = self.mag
            print(f"Mag norm (~50 uT): {math.sqrt(mx**2 + my**2 + mz**2) * 1e6}") # used for checking magnetic disturbances/offsets
            bx = (my*math.cos(self.pitch))-(mz*math.sin(self.pitch))
            by = mx*math.cos(self.roll) + my*math.cos(self.roll)*math.sin(self.pitch) + mz*math.cos(self.roll)*math.cos(self.pitch)
            mag_accel_yaw = math.atan2(-by,bx) + math.pi
        else:
            mag_accel_yaw = self.yaw
        
        # TODO: Fuse gyro, mag, and accel derivations in complemtnary filter
        self.roll = self.alpha*gyro_roll + (1-self.alpha)*accel_roll
        self.pitch = self.alpha*gyro_pitch + (1-self.alpha)*accel_pitch
        self.yaw = self.alpha*gyro_yaw + (1-self.alpha)*mag_accel_yaw
        self.posx = self.posx + self.vx * dt
        self.posy = self.posy + self.vy * dt
        # TODO: Publish to pose_estimate topic
        Position = Vector3()
        Position.x = self.posx
        Position.y = self.posy
        Position.z = self.yaw
        self.publisher_pose_estimate.publish(Position)

    def mag_callback(self, data):
        # TODO: Assign self.mag to the magnetometer data points
        self.mag = data.magnetic_field
        
    
    
def main():
    rclpy.init(args=None)
    node = CompFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()