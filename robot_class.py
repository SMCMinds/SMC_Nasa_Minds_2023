from math import *
import random
import numpy as np



### ------------------------------------- ###
# Below, is the robot class
#
# This robot lives in 2D, x-y space, and its motion is
# pointed in a random direction, initially.
# It moves in a straight line until it comes close to a wall
# at which point it stops.
#
# For measurements, it  senses the x- and y-distance
# to landmarks. This is different from range and bearing as
# commonly studied in the literature, but this makes it much
# easier to implement the essentials of SLAM without
# cluttered math.
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
        #position x,y,theta
        self.vel = np.array([[4],[0]])
        self.pos = [world_size * random.random(),world_size * random.random(),0]
        self.record_movement = [[self.pos[0], self.pos[1], self.pos[2]]] #record_movement[x,y,orientation]
        
    def lim_angle(angle):
        while angle < 0:
           angle += 2*pi
        while angle > (2*pi):
           angle -= 2*pi
        return angle
            
    def move(self):
        
        np_follower_array = np.array(self.pos)
        transform = np.array([[cos(self.pos[2]), 0], 
                             [sin(self.pos[2]), 0],
                             [0, 1]])
        
        ###Put new_pos in move function because it is moving now ####
        ### Make in terms of event frame###
        ####Add Collision Avoidance########
        #while self.check_if_collide(x,y) or x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
        new_pos = np_follower_array +  np.matmul(transform, self.vel)
        self.pos = new_pos.tolist()
        self.record_movement.append(self.pos)
        
        #old code#
    ''' random.seed()
        #initialize
        orientation = self.orientation + random.uniform(-2,2)/5
        dx = cos(orientation) * self.distance
        dy = sin(orientation) * self.distance
        x = self.x + dx
        y = self.y + dy
        anti_loop_counter = 0

        #check if new coordinate goes out of bounds or hits a landmark
        while self.check_if_collide(x,y) or x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            #### In any collision, the robot will automatically turn left
            orientation = orientation + 0.05 
            dx = cos(orientation) * self.distance
            dy = sin(orientation) * self.distance
            if anti_loop_counter >= 10:
                dx *=2
                dx *=2
            x = self.x + dx
            y = self.y + dy

            anti_loop_counter+=1

        #assignment
        self.x = x
        self.y =  y
        self.orientation = orientation
        self.record_movement.append([self.x,self.y, self.orientation])
        '''
    
    def wing_pos(self, leader):
        #Ensure this function is only called when leader is ahead
        #0 for error
        #1 for right
        #2 for left
        dist_x = leader.pos[0] - self.pos[0] #x
        dist_y = leader.pos[1] - self.pos[1] #y
        leading_theta = self.lim_angle(leader.record_movement[-1][2])
        if leading_theta < pi/2:
            if dist_x < 0 and dist_y < 0: #Q3
                pos_theta = abs(atan2(dist_y,dist_x))
                if pos_theta > leading_theta:
                    return 1
                else: return 2
            if dist_x > 0 and dist_y < 0: #Q4
                return 1
            if dist_x < 0 and dist_y > 0: #Q2
                return 2
        elif leading_theta < pi:
            if dist_x > 0 and dist_y > 0: #Q1
                return 1
            if dist_x < 0 and dist_y < 0: #Q3
                return 2
            if dist_x > 0 and dist_y < 0: #Q4
                pos_theta = pi + atan2(dist_y,dist_x)
                if pos_theta > leading_theta:
                    return 1
                else: return 2       
        elif leading_theta < 3*pi/2:
            if dist_x > 0 and dist_y > 0: #Q1
                pos_theta = pi + atan2(dist_y,dist_x)
                if pos_theta > leading_theta:
                    return 1
                else: return 2
            if dist_x < 0 and dist_y > 0: #Q2
                return 2
            if dist_x > 0 and dist_y < 0:
                return 1             
        elif leading_theta < 2*pi:
            if dist_x > 0 and dist_y > 0: #Q1
                return 2
            if dist_x < 0 and dist_y > 0: #Q2
                pos_theta = pi + atan2(dist_y,dist_x)
                if pos_theta > leading_theta:
                    return 1
                else: return 2
            if dist_x < 0 and dist_y < 0: #Q3
                return 1
            
        return 0
                            
    def goal_position(self,spacing, leader):
        left_or_right = self.wing_pos(leader)
        leading_theta = self.lim_angle(leader.record_movement[-1][2])
        angle_spacing = pi/4
        if left_or_right == 1: #right wing
            angle_of_following = leading_theta - pi/2 -angle_spacing
            return -1*spacing*cos(angle_of_following), spacing*sin(angle_of_following)
        if left_or_right == 2:
            angle_of_following = leading_theta + pi/2 + angle_spacing
            return spacing*cos(angle_of_following), spacing*sin(angle_of_following)
        else:
            print('goal position error')
            return 0
            
        
        
    def follower_pos(self, leader, wing_pos, dt = 0.1):
        goal_x, goal_y = self.goal_position(5,leader)
        goal_theta = leader.record_movement[-1][2]
        np_follower_array = np.array(self.pos)
        error_frame = np.array([[goal_x - self.record_movement[-1][0], goal_y - self.record_movement[-1][1],
                       goal_theta - self.record_movement[-1][2]]])
        follower_heading_error = lambda x: np.matmul([[cos(self.record_movement[x][2]), sin(self.record_movement[x][2], 0)],
                        [-sin(self.record_movement[x][2]), cos(self.record_movement[x][2]), 0]],
                                           error_frame.transpose())
        
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
        
        
        
        
        

    def if_collide_with_landmark(self, x, y):
        for index in range(len(self.landmarks)):
            dist_new_x = self.landmarks[index][0] - x
            dist_new_y = self.landmarks[index][1] - y
            # check the robot crosses the landmark position
            if((abs(dist_new_y) < self.measurement_range/2) and (abs(dist_new_x) < self.measurement_range/2)):
                return True
        return False

    # --------
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #

    ###############CHANGE#############
    # Places the landmark in front of the robot

    def detect_landmarks(self):
        orientation = random.random() * 2.0 * pi
        obstacles = self.simulate_sense()
        for i in range(len(obstacles)):
            if obstacles[i][0] < self.measurement_range or obstacles[i][1] < self.measurement_range:  # Left off here
                # obstacle_x = self.x * cos(orientation)
                # obstacle_y = self.y * sin(orientation)
                self.landmarks.append(
                    [self.record_movement[-1][0] + obstacles[i][0], self.record_movement[-1][1] + obstacles[i][1]])

    ##############CHANGE###############

    ##########ONLY FOR SIMULATION#############

    def simulate_sense(self):
        measurements = []
        # TODO: iterate through all of the landmarks in a world
        for index in range(len(self.world_landmarks)):
            # distance between landmark and rover
            dist_x = self.world_landmarks[index][0] - self.record_movement[-1][0]
            dist_y = self.world_landmarks[index][1] - self.record_movement[-1][1]
            # check if landmark is in range
            if(abs(dist_x) < self.measurement_range and abs(dist_y) < self.measurement_range):
                measurements.append([dist_x, dist_y])
        # TODO: return the final, complete list of measurements
        return measurements

    # called when print(robot) is called; prints the robot's location

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]' % (self.x, self.y)


####### END robot class #######
