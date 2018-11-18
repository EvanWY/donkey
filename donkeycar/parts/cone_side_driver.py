from donkeycar import util
import numpy as np
import matplotlib
import cv2

class ConeSideDriver:
    def __init__(self):
        self.last_shift = 0
        self.prev_target_pose = 0

    def run(self, detections):
        target = None
        for detection in detections:
            if target is None or (target['n1'] - target['n0']) < (detection['n1'] - detection['n0']):
                target = detection
        
        if target is None:
            targetPos = self.prev_target_pose + 1
        else:
            targetPos = (target['n1'] + target['n0']) / 2

        shift = (targetPos - 150) / 150.0

        steer = shift * 2 + (shift-self.last_shift) * 0.1

        self.last_shift = shift
        self.prev_target_pose = targetPos

        return steer, 0.2

