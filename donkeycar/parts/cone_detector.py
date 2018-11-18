import numpy as np
import matplotlib
import cv2

class ConeDetector:
    def run(self, img_arr):
        img_vis = np.copy(img_arr)

        nclip = int(img_arr.shape[0] * 0.35)
        img_clip = img_arr[nclip:-nclip,:,:]
        yuv = cv2.cvtColor(img_clip, cv2.COLOR_BGR2YUV)
        yuv_channel_v = yuv[:,:,2]

        chv_larger_threshold = yuv_channel_v > 160
        chv_col_orange_count = np.sum(chv_larger_threshold, axis=0)

        print (chv_col_orange_count)

        chv_col_has_orange = (chv_col_orange_count > 0).tolist()
        chv_col_has_orange.append(False)
        curr_begin = 0
        curr_end = 0
        detections = []
        for n in range(1, len(chv_col_has_orange)):
            if (not chv_col_has_orange[n-1]) and chv_col_has_orange[n]:
                curr_begin = n
            elif chv_col_has_orange[n-1] and not chv_col_has_orange[n]:
                curr_end = n
                if curr_end > curr_begin + 1:
                    detections.append({'n0':curr_begin, 'n1':curr_end})
        
        ## debug image
        for detection in detections:
            n0 = detection['n0']
            n1 = detection['n1']
            m0 = 60 - (n1-n0)
            m1 = 60 + (n1-n0)
            cv2.rectangle(img_vis, (n0, m0), (n1, m1), [255,0,0], thickness=1, lineType=8, shift=0)

        return detections, yuv_channel_v


if __name__ == '__main__':
    import glob
    img_names = glob.glob('temp/*')
    img_names.sort()
    #img_names = ['temp/{}.jpg'.format(i) for i in range(12)]

    for img_name in img_names:
        img = cv2.imread(img_name)
        detector = ConeDetector()

        img_out, detections = detector.run(img)

        img_out_x3 = np.stack([img_out,img_out,img_out],axis=2)
        
        for detection in detections:
            n0 = detection['n0']
            n1 = detection['n1']
            m0 = 60 - (n1-n0)
            m1 = 60 + (n1-n0)
            #cv2.line(img, (y, x-pix), (y, x+pix), [255,0,0], thickness=1, lineType=8, shift=0)
            #cv2.line(img, (y-pix, x), (y+pix, x), [255,0,0], thickness=1, lineType=8, shift=0)
            cv2.rectangle(img, (n0, m0), (n1, m1), [255,0,0], thickness=1, lineType=8, shift=0)
        
        cv2.imshow('Test image', np.vstack([img, img_out_x3]))
        cv2.waitKey(0)

    cv2.destroyAllWindows()