from donkeycar import util
import numpy as np
import matplotlib
import cv2

class LineFollower:
    def run(self, img_arr):
        #img = np.copy(img_arr)
        #img = img[...,::-1]
        #img = matplotlib.colors.rgb_to_hsv(img / 255)

        #img[:,:,0] = img[:,:,0]
        #img[:,:,1] = (img[:,:,1] > 0.5) * 1
        #img[:,:,2] = (img[:,:,2] > 0.5) * 1        
        
        # img[:,:,0] = img[:,:,0]
        # img[:,:,1] = img[:,:,0]
        # img[:,:,2] = img[:,:,0]
        
        #img = (matplotlib.colors.hsv_to_rgb(img) * 255).astype(np.uint8)
        #img = img[...,::-1]

        img = cv2.cvtColor(img_arr, cv2.COLOR_BGR2HSV)
        # img[:,:,1] = 255
        #img[:,:,2] = 128
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

        return 0, 0, img_arr

