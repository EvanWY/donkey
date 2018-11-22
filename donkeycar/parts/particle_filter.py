import math
import random

PARTICLE_INIT_X_Y_SCALE = 100
PARTICLE_NUMBER = 1000
FORWARD_NOISE = 0.05
TURN_NOISE = 0.05
SENSE_NOISE = 5.0

landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
             [80.0, 20.0], [80.0, 50.0]]

class Particle:
    def __init__(self, turn_noise, forward_noise, sense_noise):
        self.x = random.random() * PARTICLE_INIT_X_Y_SCALE
        self.y = random.random() * PARTICLE_INIT_X_Y_SCALE
        self.theta = random.random() * 2.0 * math.pi
        self.weight = 1.0

        self.turn_noise = turn_noise
        self.forward_noise = forward_noise
        self.sense_noise = sense_noise
    
    def clone(self):
        new_particle = self.__class__(self.turn_noise, self.forward_noise, self.sense_noise)
        new_particle.x = self.x
        new_particle.y = self.y
        new_particle.theta = self.theta
        new_particle.weight = self.weight
        return new_particle

    def move(self, turn, forward):
        self.theta += turn + random.gauss(0.0, self.turn_noise)
        self.theta %= 2.0 * math.pi

        forward_dist = forward + random.gauss(0.0, self.forward_noise)
        self.x += forward_dist * math.cos(self.theta)
        self.y += forward_dist * math.sin(self.theta)
    
    def update_weight(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        self.weight = prob

    @staticmethod
    def gaussian(mu, sigma, x):
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))


class ParticleFilter:
    def __init__(self):
        self.particles = []
        for _ in range(PARTICLE_NUMBER):
            self.particles.append(Particle(TURN_NOISE, FORWARD_NOISE, SENSE_NOISE))

    def run(self, measurement, prev_steer, prev_throttle):
        delta_time = 0.033
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


def visualization(robot, step, p, pr):
    import matplotlib.pyplot as plt
    from math import *
    world_size = 100.0
    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))
    mng = plt.get_current_fig_manager()
    mng.window.maxsize(700,700)
 
    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])
 
    # draw particles
    for ind in range(len(p)):
 
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
 
    # robot's location
    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)
 
    # robot's theta
    arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.theta), 2*sin(robot.theta), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)
 
    figname = "figure_" + str(step) + ".png"
    plt.savefig(figname)
    print('Saved',figname)
    plt.clf()
    #plt.show()


class Robot(Particle):
    def sense(self):
        z = []
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)
        return z

if __name__ == '__main__':
    delta_time = 0.033

    particle_filter = ParticleFilter()
    robot = Robot(0,0,SENSE_NOISE)
    robot.x = 55
    robot.y = 30
    robot.theta = 120

    for i in range(30):
        if i < 3 or i % 5 == 0:
            visualization(robot, i, particle_filter.particles, [])
        robot.move(6 * delta_time, 150 * delta_time)
        meas = robot.sense()
        particle_filter.run(meas, 6, 150)
