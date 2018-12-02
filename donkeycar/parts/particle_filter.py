from math import *
import random
import numpy as np

DEBUG = True

PARTICLE_INIT_X_Y_SCALE = 10
PARTICLE_NUMBER = 1000
FORWARD_NOISE = 0.5
TURN_NOISE = 0.5

SENSE_PHI_STDDEV = 0.13
SENSE_LOG_R_STDDEV = 0.01

MAX_POLAR_DIST_ALLOW = 5.0
POLAR_DIST_PHI_SCALE_FACTOR = 450

SENSE_DISTANCE_RANGE = [1, 100]
SENSE_FOV_RANGE = [-0.558, 0.558]

# landmarks = [[20.0, 20.0], [34.0, 28.0], [50.0, 60.0],
#              [75.0, 90.0], [54.0, 70.0], [60.0, 80.0],
#              [70.0, 20.0], [66.0, 33.0]]
landmarks = [
    [0.2745, 7.1675],
    [4.087, 7.137],
    [2.379, 4.1175],
    [0.732, 0.915],
    [4.1175, 0.915],
]


class Particle:
    def __init__(self, turn_noise, forward_noise):
        self.x = random.random() * PARTICLE_INIT_X_Y_SCALE * 0.5 + PARTICLE_INIT_X_Y_SCALE * 0.25
        self.y = random.random() * PARTICLE_INIT_X_Y_SCALE * 0.5 + PARTICLE_INIT_X_Y_SCALE * 0.25
        self.theta = random.random() * 2.0 * pi
        self.weight = 1.0

        self.turn_noise = turn_noise
        self.forward_noise = forward_noise
    
    def clone(self):
        new_particle = self.__class__(self.turn_noise, self.forward_noise)
        new_particle.x = self.x
        new_particle.y = self.y
        new_particle.theta = self.theta
        new_particle.weight = self.weight
        return new_particle

    def move(self, turn, forward):
        self.theta += turn + random.gauss(0.0, self.turn_noise)
        self.theta %= 2.0 * pi

        forward_dist = forward + random.gauss(0.0, self.forward_noise)
        self.x += forward_dist * cos(self.theta)
        self.y += forward_dist * sin(self.theta)
    
    # measurement: [[r, phi], [r, phi], .. ]
    def update_weight(self, measurement):
        prob = 1.0
        cosT = cos(self.theta)
        sinT = sin(self.theta)

        polar_landmarks = []

        for lm in landmarks:
            dx = lm[0] - self.x
            dy = lm[1] - self.y
            lm_x = dx * cosT + dy * sinT
            lm_y = - dx * sinT + dy * cosT
            r = sqrt(lm_x ** 2 + lm_y ** 2)
            phi = atan2(lm_y, lm_x)
            polar_landmarks.append([r, phi, False])

        def polar_dist(lhs, rhs):
            phi_dist = (lhs[1] - rhs[1]) ** 2
            phi_dist *= POLAR_DIST_PHI_SCALE_FACTOR
            
            r_dist = 0
            if (lhs[0] > rhs[0]):
                r_dist = lhs[0] / rhs[0]
            else:
                r_dist = rhs[0] / lhs[0]
            r_dist -= 1
            
            return phi_dist + r_dist
        
        def calc_ghost_measurement_prob(meas):
            # x1, x2 = SENSE_DISTANCE_RANGE
            # y1, y2 = 0.5, 1

            # x = meas[0]
            # y = (x - x1) / (x2 - x1) * (y2-y1) + y1

            # return max(min(y, y1), y2)
            r, phi = meas[0], meas[1]
            ghost_distance_to_sensor_max_range = polar_dist(meas, [SENSE_DISTANCE_RANGE[1], phi])
            return 1.0 / exp(ghost_distance_to_sensor_max_range)
        
        for meas in measurement:
            min_dist = None
            min_dist_landmark = None
            for lm in polar_landmarks:
                dist = polar_dist(lm, meas)
                if dist > MAX_POLAR_DIST_ALLOW:
                    continue
                elif not (min_dist and min_dist < dist):
                    min_dist = dist
                    min_dist_landmark = lm

            if min_dist:
                prob *= (1.0 / exp(min_dist))
                min_dist_landmark[2] = True
            else:
                prob *= calc_ghost_measurement_prob(meas)
        
        def calc_missing_landmark_prob(landmark):
            r, phi = landmark[0], landmark[1]
            if phi < SENSE_FOV_RANGE[0] or phi > SENSE_FOV_RANGE[1] or r < SENSE_DISTANCE_RANGE[0] or r > SENSE_DISTANCE_RANGE[1]:
                return 1.0
            else:
                landmark_distance_to_sensor_max_range = polar_dist(landmark, [SENSE_DISTANCE_RANGE[1], phi])
                return 1.0 / exp(landmark_distance_to_sensor_max_range)

        for lm in polar_landmarks:
            if lm[2] == False:
                prob *= calc_missing_landmark_prob(lm)

        self.weight = prob
            
    # @staticmethod
    # def gaussian(sigma, x):
    #     return exp(- (x ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


class ParticleFilter:
    def __init__(self):
        self.particles = []
        for _ in range(PARTICLE_NUMBER):
            self.particles.append(Particle(TURN_NOISE, FORWARD_NOISE))
    
    @staticmethod
    def cone_detection_to_measurement(detection):
        measurement = []
        for d in detection:
            n1, n0 = d['n1'], d['n0']
            center = (n1 + n0) * 0.5
            r = 0.305 * 65/(n1-n0)
            phi = atan2(-(center - 80), 128.09)
            measurement.append([r, phi])
        print (measurement)
        return measurement
        

    def run(self, detection, prev_steer, prev_throttle, delta_time):
        if not prev_steer:
            prev_steer = 0
        if not prev_throttle:
            prev_throttle = 0
        
        measurement = self.cone_detection_to_measurement(detection)

        turn = prev_steer * delta_time
        forward = prev_throttle * delta_time
        max_weight = None
        for p in self.particles:
            p.move(turn, forward)
            p.update_weight(measurement)
            if not (max_weight and p.weight <= max_weight):
                max_weight = p.weight
        
        prev_particles = self.particles
        self.particles = []
        index = int(random.random() * PARTICLE_NUMBER)
        beta = 0.0
        for _ in range(PARTICLE_NUMBER):
            beta += random.random() * 2.0 * max_weight
            while beta > prev_particles[index].weight:
                beta -= prev_particles[index].weight
                index = (index + 1) % PARTICLE_NUMBER
            
            new_particle = prev_particles[index].clone()
            self.particles.append(new_particle)
        
        average_pose = [0, 0, 0]
        for p in self.particles:
            average_pose[0] += p.x
            average_pose[1] += p.y
            average_pose[2] += p.theta
        average_pose[0] /= PARTICLE_NUMBER
        average_pose[1] /= PARTICLE_NUMBER
        average_pose[2] /= PARTICLE_NUMBER
        
        if DEBUG:
            print (average_pose)
            img = np.zeros([100, 100, 3]).astype(np.uint8)
            for l in landmarks:
                xx= int(l[0] * 10)
                yy= int(l[1] * 10)
                img[xx, yy, 1] = 255
                img[xx+1, yy, 1] = 255
                img[xx, yy+1, 1] = 255
                img[xx+1, yy+1, 1] = 255

            xx= int(average_pose[0] * 10)
            yy= int(average_pose[1] * 10)
            if xx >= 0 and xx < 99 and yy >= 0 and yy < 99:
                img[xx, yy, 0] = 255
                img[xx+1, yy, 0] = 255
                img[xx, yy+1, 0] = 255
                img[xx+1, yy+1, 0] = 255
            else:
                print ('out of range')

        
        return img


def visualization(robot, step, p, pr, meas):
    import matplotlib.pyplot as plt
    world_size = 100.0
    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))
    #mng = plt.get_current_fig_manager()
    #mng.window.maxsize(700,700)
 
    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])
 
    # draw particles
    for ind in range(len(p)):
        if ind % 50 != 3:
            continue
 
        # particle
        circle = plt.Circle((p[ind].x, p[ind].y), 1., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's theta
        arrow = plt.Arrow(p[ind].x, p[ind].y, 2*cos(p[ind].theta), 2*sin(p[ind].theta), alpha=1., facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)
 
    # draw resampled particles
    for ind in range(len(pr)):
 
        # particle
        #circle = plt.Circle((pr[ind].x, pr[ind].y), 1., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        circle = plt.Circle((pr[ind].x, pr[ind].y), 0.5, facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's theta
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 2*cos(pr[ind].theta), 2*sin(pr[ind].theta), alpha=1., facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)
 
    # fixed landmarks of known locations
    for lm in landmarks:
        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(circle)
    
    for m in meas:
        r, phi = m[0], m[1]
        sum_phi_theta = phi + robot.theta
        x = r * cos(sum_phi_theta) + robot.x
        y = r * sin(sum_phi_theta) + robot.y

        circle = plt.Circle((x, y), 0.5, facecolor='#00cc00', edgecolor='#003300')
        plt.gca().add_patch(circle)
 
    # robot's location
    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)
 
    # robot's theta
    arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.theta), 2*sin(robot.theta), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)
 
    figname = "figure_" + str(step) + ".png"
    plt.savefig(figname)
    #print('Saved',figname)
    plt.clf()
    #plt.show()


class Robot(Particle):
    def sense(self):
        z = []
        cosT = cos(self.theta)
        sinT = sin(self.theta)
        for lm in landmarks:
            dx = lm[0] - self.x
            dy = lm[1] - self.y
            lm_x = dx * cosT + dy * sinT
            lm_y = - dx * sinT + dy * cosT
            r = sqrt(lm_x ** 2 + lm_y ** 2)
            phi = atan2(lm_y, lm_x)

            if phi < SENSE_FOV_RANGE[0] or phi > SENSE_FOV_RANGE[1] or r < SENSE_DISTANCE_RANGE[0] or r > SENSE_DISTANCE_RANGE[1]:
                continue
            else:
                #z.append([exp(self.gaussian(0.5, log(r))), self.gaussian(0.01, phi)])
                #z.append([r, phi])
                z.append([exp(random.gauss(log(r), 0.69)), random.gauss(phi, 0.1)])
        return z
            

if __name__ == '__main__':
    delta_time = 0.033

    particle_filter = ParticleFilter()
    robot = Robot(0,0)
    robot.x = 30
    robot.y = 30
    robot.theta = pi * 0.2

    steer, throttle = 3, 150

    meas = []
    for i in range(500):

        dx = 50 - robot.x
        dy = 50 - robot.y
        lm_x = dx * cos(robot.theta) + dy * sin(robot.theta)
        lm_y = - dx * sin(robot.theta) + dy * cos(robot.theta)
        r = sqrt(lm_x ** 2 + lm_y ** 2)
        phi = atan2(lm_y, lm_x)
        v = (phi / pi) * (r / 75.0) * 10
        steer = 50.0 / (1.0 + e ** (-v)) - 25.0
        throttle = 200 - 3.0 * r

        sum_dist = 0
        for p in particle_filter.particles:
            dist = sqrt((p.x - robot.x) ** 2 + (p.y - robot.y) ** 2)
            sum_dist += dist
        
        s = ''
        s += str(i)
        s += '\t'
        s += str(int(sum_dist * 0.01))
        s += '\t'
        c = int((sum_dist * 0.001 * 0.02) * 70)
        for i2 in range(c):
            s += '-'
        print (s)

        # if i < 5 or i % 10 == 0:
        #     visualization(robot, i, particle_filter.particles, [], meas)
            
        robot.move(steer * delta_time, throttle * delta_time)
        meas = robot.sense()
        particle_filter.run(meas, steer, throttle, delta_time)