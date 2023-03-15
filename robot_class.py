
from math import *
import random
import numpy as np



#
class robot:

    # --------
    # init:
    #   creates a robot with the specified parameters and initializes
    #   the location (self.x, self.y) to the center of the world
    #
    # world_landmarks only for this simulation
    def __init__(self, world_size=100.0, measurement_range=30.0, world_landmarks=0):
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.landmarks = []  # landmarks[x][y] seen by rover
        self.world_landmarks = world_landmarks
        self.distance = 1.0  # The size of the step that the robot takes
        self.vel = np.array([2.0,0.0]) # [velocity, angular velocity]
        self.pos = [world_size * random.random(),world_size * random.random(),0] #position [x,y,theta] , Made purely so computer can access easier
        self.record_movement = [[self.pos[0], self.pos[1], self.pos[2]]] #record_movement[x,y,theta]
        self.world_map = np.zeros((int(world_size), int(world_size)), bool)
        
    #Limit the angular orientation between 0 < theta < pi
    def lim_angle(angle):
        while angle < 0:
           angle += 2*pi
        while angle > (2*pi):
           angle -= 2*pi
        return angle
            
    #Movement Algorithm
    def move(self):
        random.seed()
        #initialize
        orientation = self.orientation + random.uniform(-2,2)/5
        dx = cos(orientation) * self.distance
        dy = sin(orientation) * self.distance
        x = self.x + dx
        y = self.y + dy
        anti_loop_counter = 0
        
        #E(t) is all the errors. Multiply the errors with the PID controller
        #error, integral error, derivative error
        #time based pid
        '''E_t = lambda x: [follower_heading_error[x], following_heading_error[x]*dt, following_heading_error[x]/dt]
            H_t = np.array[[E_t[0], 0, 0],
                [0, E_t[1], follower_heading_error[2]]]'''
                
        #######Might Have an Error from the lack of array size############
        if len(self.record_movement) < 3:
            E_k = lambda x: [follower_heading_error(-1)[x], 
                         following_heading_error(-1)[x],
                         following_heading_error(-1)[x]]
        else:
            E_k = lambda x: [follower_heading_error(-1)[x] - follower_heading_error(-2)[x], 
                         following_heading_error(-1)[x],
                         following_heading_error(-1)[x] - 2*following_heading_error(-2)[x] + following_heading_error(-3)[x]]
        ##################################################################
        
        H_k = np.array([[E_k(0), 0, 0],
                [0, E_k(1), follower_heading_error(-1)[2]]])
        #Ki, Kp, Kd
        K_e = [1,10,0.001]
        #[[v],[w]]
        vel = np.matmul(K_e, H_k)
        self.vel += vel
        transform = np.array([[cos(self.record_movement[-1][2]), 0], 
                             [sin(self.record_movement[-1][2]), 0],
                             [0, 1]])
        self.world_map[int(self.x)][int(self.y)] = 1

    #Check for collision with obstacles
    def if_collide_with_landmark(self, x, y):
        for index in range(len(self.landmarks)):
            dist_new_x = self.landmarks[index][0] - x
            dist_new_y = self.landmarks[index][1] - y
            # check the robot crosses the landmark position, Avoidance when obstacle is half of detection range
            if((abs(dist_new_y) < self.measurement_range/2) and (abs(dist_new_x) < self.measurement_range/2)):
                return True
        return False

##########Change???###############
    # Places the landmark in front of the robot
    def detect_landmarks(self):
        orientation = random.random() * 2.0 * pi
        obstacles = self.simulate_sense()
        for i in range(len(obstacles)):
            if obstacles[i][0] < self.measurement_range or obstacles[i][1] < self.measurement_range:  # Left off here
                self.landmarks.append(
                    [self.record_movement[-1][0] + obstacles[i][0], self.record_movement[-1][1] + obstacles[i][1]])
##############CHANGE###############

    def simulate_sense(self):
        measurements = []
        # TODO: iterate through all of the landmarks in a world
        for index in range(len(self.world_landmarks)):
            # distance between landmark and rover
            dist_x = self.world_landmarks[index][0] - self.pos[0]
            dist_y = self.world_landmarks[index][1] - self.pos[1]
            # check if landmark is in range
            if(abs(dist_x) < self.measurement_range and abs(dist_y) < self.measurement_range):
                measurements.append([dist_x, dist_y])
        # TODO: return the final, complete list of measurements
        return measurements

    # called when print(robot) is called; prints the robot's location

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]' % (self.pos[0], self.pos[1])

    ###### Adding records of another robot to this one. #####
    def add_records(self,robot2):
        self.world_map = self.world_map + robot2.world_map


####### END robot class #######

<<<<<<< HEAD

####### END robot class #######



####### Create its own map #######

# This will help two things-
# a) Easy to print out/ graph
# b) Easy to add/store other records. Avoids unlimited records as well.

# Since it's an np matrix, we can simply just add two 'maps' together, quickly.
# the '+' sign is just a quick-hand 'or' operator.




# ob = robot(100, 5, 6)
# print(ob.record_movement[-1][0])