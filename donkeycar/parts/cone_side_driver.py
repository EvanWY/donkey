from donkeycar import util
import numpy as np
import matplotlib
import cv2

class ConeSideDriver:
    def run(self, detections):
        target = None
        for detection in detections:
            if target is None or (target['n1'] - target['n0']) < (detection['n1'] - detection['n0']):
                target = detection
        
        if target is None:
            targetPos = 300
        else:
            targetPos = (target['n1'] + target['n0']) / 2

        shift = (targetPos - 150) / 150.0

        steer = shift*4

        return steer, 0.2

