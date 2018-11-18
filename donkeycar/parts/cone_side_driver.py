from donkeycar import util
import numpy as np
import matplotlib
import cv2

class ConeSideDriver:
    def run(self, detections):
        target = None
        for detection in detections:
            if target is None or target['n1'] - target['n0'] < detection['n1'] - detection['n0']
        
        if target is None:
            targetPos = 160
        else:
            targetPos = (target['n1'] + target['n0']) / 2

        shift = (targetPos - 150) / 150.0

        steer = shift

        return 0, 0.7

