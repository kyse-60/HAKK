import os
import sys
import time
import cv2 as cv # type: ignore
import numpy as np # type: ignore

sys.path.insert(0, "library")
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

from pycoral.adapters.common import input_size # type: ignore
from pycoral.adapters.detect import get_objects # type: ignore
from pycoral.utils.dataset import read_label_file # type: ignore
from pycoral.utils.edgetpu import make_interpreter # type: ignore
from pycoral.utils.edgetpu import run_inference # type: ignore

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'ARROW.tflite'
label_name = 'arrow.txt'
model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 9

interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()
labels = read_label_file(label_path)
inference_size = input_size(interpreter)

# Initialize car
rc = racecar_core.create_racecar()

# Global variables
contour_center = None
contour_area = 0

speed = 0
angle = 0 
prev_error=0

sign = ""

center = 160

area = 0

tag = {1:'Left' , 2: 'Right'}
type = "nah"
rcount = 0
lcount = 0
count = 0
last_type = "nah"
changed = False
                
# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if obj.score > 0.6:
            
            print(f"{tag[obj.id+1]}")
            
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

            cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv.putText(cv2_im, label, (x0, y0+30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            break
    return cv2_im

# Called once on start
def start():
    global changed 
    
    rc.drive.set_speed_angle(0, 0)
    changed = False

# Called every frame
def update():
    time_cp1 = time.time()
    
    global speed, angle  
    global prev_error
    global type 
    global count 
    global changed

    error = center - 160

    # get the change in time
    dt=rc.get_delta_time()

    # proportional factor and derivative factor
    kd = 0.9 * 0
    kp = 0.0027
    # Calculate the change in error
    derror=error-prev_error
    
    # Combine the proportional term and the derivative term to compute the angle 
    angle = kp*error + derror/dt*kd
    # print(angle)
    # Clamp the angle 
    prev_angle = angle
    dangle = angle - prev_angle

    if type == "right" and changed:
        speed = 0.55
        print("count", count)
        count += 1
        if count < 100:
            cangle = -0.8
        elif count < 140:
            cangle = 0.6
        else: 
            cangle = 0
    elif type == "left" and changed: 
        speed = 0.55
        print("count ", count)
        count += 1
        if count < 100:
            cangle = 0.8
        elif count < 140:
            cangle = -0.6
        else:
            cangle = 0
    else: 
        cangle = 0
        speed = 0
    

    # if area > 17000:
    #     rc.drive.set_speed_angle(0, 0)
    # else:
    #     pass
    print(f'angle {cangle}')
    rc.drive.set_speed_angle(speed, cangle)
        
    prev_error=error
    #############################################################################################

# Called once per second
def update_slow():
    global center
    global sign
    global area
    global rcount , lcount 
    global type 
    global last_type
    global count
    global changed
    frame = rc.camera.get_color_image()
    
    if frame is not None:
        rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb_image = cv.resize(rgb_image, inference_size)
        
        run_inference(interpreter, rgb_image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        image = append_objs_to_img(frame, inference_size, objs, labels)

        rc.display.show_color_image(image)
        if len(objs) == 0:
            sign = ""
        for obj in objs:
            if obj.score > 0.6:
                sign = tag[obj.id+1]
                if sign == "Right":
                    rcount += 1
                    if(rcount > 10): 
                        type = "right"
                        print("type now right")
                else:
                    rcount = 0
                if sign == "Left":
                    lcount += 1
                    if(lcount > 10): 
                        type = "left"
                        print("type now left")          
                else:
                    lcount = 0
            if last_type != type and last_type != "nah":
                count = 0
                changed = True
        
            last_type = type 
        
                # if sign == "Car":
                #     loc = (obj.bbox.xmin + obj.bbox.xmax)//2
                #     area = (obj.bbox.xmin - obj.bbox.xmax ) * (obj.bbox.ymin - obj.bbox.ymax )
                #     center = loc
            print(f"{tag[obj.id+1]}")
            break
            sign = ""

if __name__ == "__main__":
    rc.set_update_slow_time(0.1)
    rc.set_start_update(start, update, update_slow)
    rc.go()