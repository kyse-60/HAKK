import sys
import math 
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

def start():
    pass

def update():
    wall_follower()

def wall_follower():
    RAY_FROM_OFFSET = 50.0
    WINDOW = 3.0
    kp = 0.0042
    speed = 1

    scan = rc.lidar.get_samples()

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

    rc.drive.set_speed_angle(speed, angle)
    print(error)

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()