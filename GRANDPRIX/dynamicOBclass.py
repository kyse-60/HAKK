# dynamic_obstacle.py

import time
import cv2 as cv  # type: ignore
import numpy as np  # type: ignore
import sys

sys.path.insert(0, "library")
import racecar_core  # type: ignore
import racecar_utils as rc_utils  # type: ignore

from pycoral.adapters.common import input_size  # type: ignore
from pycoral.adapters.detect import get_objects  # type: ignore
from pycoral.utils.dataset import read_label_file  # type: ignore
from pycoral.utils.edgetpu import make_interpreter  # type: ignore
from pycoral.utils.edgetpu import run_inference  # type: ignore


class DynamicObstacle:
    def __init__(
        self,
        rc,
        model_dir="models",
        model_name="ARROW.tflite",
        label_name="arrow.txt",
        score_thresh=0.1,
        num_classes=9,
        slow_interval=0.1,
    ):
        self.rc = rc

        # Load model + labels
        model_path = f"{model_dir}/{model_name}"
        label_path = f"{model_dir}/{label_name}"
        self.interpreter = make_interpreter(model_path)
        self.interpreter.allocate_tensors()
        self.labels = read_label_file(label_path)
        self.inference_size = input_size(self.interpreter)

        # Detection/control state
        self.center = 160
        self.speed = 0.0
        self.angle = 0.0
        self.prev_error = 0.0

        self.sign = ""
        self.type = "nah"
        self.last_type = "nah"
        self.changed = False
        self.count = 0
        self.rcount = 0
        self.lcount = 0

        self.SCORE_THRESH = score_thresh
        self.NUM_CLASSES = num_classes
        self.TAG = {1: "Left", 2: "Right"}

        # PID params
        self.kp = 0.0027
        self.kd = 0.0  # original had 0.9 * 0

        # Slow inference throttle
        self._slow_interval = slow_interval
        self._last_slow = 0.0

        # Target center (fixed in original)
        self.target_center = 160

    def _append_objs_to_img(self, cv2_im, inference_size, objs, labels):
        height, width, channels = cv2_im.shape
        scale_x, scale_y = width / inference_size[0], height / inference_size[1]
        for obj in objs:
            if obj.score > 0.6:
                bbox = obj.bbox.scale(scale_x, scale_y)
                x0, y0 = int(bbox.xmin), int(bbox.ymin)
                x1, y1 = int(bbox.xmax), int(bbox.ymax)

                percent = int(100 * obj.score)
                label = "{}% {}".format(percent, labels.get(obj.id, obj.id))

                cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2_im = cv.putText(
                    cv2_im, label, (x0, y0 + 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2
                )
                break
        return cv2_im

    def _update_slow(self):
        """Run inference + update sign/type state. Should be called periodically."""
        frame = self.rc.camera.get_color_image()
        if frame is None:
            return

        rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb_image = cv.resize(rgb_image, self.inference_size)

        run_inference(self.interpreter, rgb_image.tobytes())
        objs = get_objects(self.interpreter, self.SCORE_THRESH)[: self.NUM_CLASSES]
        image = self._append_objs_to_img(frame, self.inference_size, objs, self.labels)
        self.rc.display.show_color_image(image)

        if len(objs) == 0:
            self.sign = ""
        for obj in objs:
            if obj.score > 0.6:
                self.sign = self.TAG.get(obj.id + 1, "")
                if self.sign == "Right":
                    self.rcount += 1
                    if self.rcount > 10:
                        self.type = "right"
                else:
                    self.rcount = 0

                if self.sign == "Left":
                    self.lcount += 1
                    if self.lcount > 10:
                        self.type = "left"
                else:
                    self.lcount = 0

            if self.last_type != self.type and self.last_type != "nah":
                self.count = 0
                self.changed = True

            self.last_type = self.type
            break  # only process first confident detection

    def update(self):
        """
        Call every frame. Returns (speed, angle) to apply.
        """
        now = time.time()
        if now - self._last_slow >= self._slow_interval:
            self._update_slow()
            self._last_slow = now

        # Compute error (fixed in original was center - 160 but center never updated by detection)
        error = self.center - self.target_center

        dt = self.rc.get_delta_time()
        derror = error - self.prev_error if dt > 0 else 0.0

        raw_angle = self.kp * error + (derror / dt) * self.kd if dt > 0 else self.kp * error

        # Decision logic based on detected type/change
        if self.type == "right" and self.changed:
            self.speed = 0.55
            self.count += 1
            if self.count < 100:
                cangle = -0.8
            elif self.count < 140:
                cangle = 0.6
            else:
                cangle = 0
        elif self.type == "left" and self.changed:
            self.speed = 0.55
            self.count += 1
            if self.count < 100:
                cangle = 0.8
            elif self.count < 140:
                cangle = -0.6
            else:
                cangle = 0
        else:
            cangle = 0
            self.speed = 0

        self.angle = rc_utils.clamp(cangle, -1.0, 1.0)
        self.prev_error = error

        return self.speed, self.angle

    def start(self):
        self.rc.drive.set_speed_angle(0, 0)
        self.changed = False
        self.count = 0
        self.last_type = "nah"
        self.type = "nah"
        self.rcount = 0
        self.lcount = 0
        self.prev_error = 0.0
        self.angle = 0.0
        self.speed = 0.0
