from donkeycar import util

class ParticleSlam:
    def run(self, img_arr, angle, throttle, timestamp):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        angle_binned, throttle = self.model.predict(img_arr)
        angle_unbinned = util.data.linear_unbin(angle_binned[0])
        return angle_unbinned, throttle[0][0]

