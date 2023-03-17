
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
        self.world_map = np.zeros((int(world_size), int(world_size)), int)
        self.behind = None
        self.behind_angle = None
        
    #Limit the angular orientation between 0 < theta < pi
    def lim_angle(self, angle):
        while angle < 0:
           angle += 2*pi
        while angle > (2*pi):
           angle -= 2*pi
        return angle
            
    #Movement Algorithm
    def move(self):
        '''
        Current Heiarchy of Movement
        1) Avoid Collision
        2) if behind another robot
        
        Add the pheromone contribution
        '''
        
        #Check if behind, if so, set the velocity
        if self.behind:
            self.follower_pos(self.behind, self.behind_angle)
        
        transform = np.array([[cos(self.pos[2]), 0], 
                            [sin(self.pos[2]), 0],
                            [0, 1]])
        
        
        #Initialize new position
        np_follower_array = np.array(self.pos)
        new_pos = np_follower_array +  np.matmul(transform, self.vel)
        anti_stuck_loop = 0
            
        #Collision Avoidance
        while self.if_collide_with_landmark(new_pos[0],new_pos[1]) or new_pos[0] < 0.0 or new_pos[0] > self.world_size or new_pos[1] < 0.0 or new_pos[1] > self.world_size:          
            #Increase angular velocity to turn
            self.vel[1] += 0.5 # 

            #Change the transform in correlation with the angular change
            transform = np.array([[cos(new_pos[2]), 0], 
                             [sin(new_pos[2]), 0],
                             [0, 1]])
            #Make new position
            new_pos = np_follower_array +  np.matmul(transform, self.vel)

            #In case of infinite loop, increase the step size
            anti_stuck_loop += 1
            if anti_stuck_loop == 10:
                self.vel[0] += 2

        #After position validated, reset angular velocity to 0
        self.vel[1] = 0

        #Convert position to Python list    
        self.pos = new_pos.tolist()
        self.lim_angle(self.pos[2])
        
        #Safe to the record
        self.record_movement.append(self.pos)
        
        #Add position to its local map
        self.world_map[int(self.pos[0])][int(self.pos[1])] = 100

    def is_behind(self, robot2, detection_angle):
        #make detection 
        angle = atan2(self.pos[1]-self.pos[1], self.pos[0] - self.pos[0])
        correct_lower = self.pos[2] - detection_angle/2
        correct_upper = self.pos[2] + detection_angle/2
        dist = sqrt((self.pos[1]-self.pos[1])**2 + (self.pos[0] - self.pos[0])**2)
        if angle > correct_lower and angle < correct_upper and dist < self.measurement_range:
            self.behind = robot2
            self.behind_angle = detection_angle
        else:
            self.behind = None
   
    #Determine whether follower will be on the left or right
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

    #Return the goal coordinates                    
    def goal_position(self,spacing, leader):
        left_or_right = self.wing_pos(leader)
        leading_theta = self.lim_angle(leader.record_movement[-1][2])
        angle_spacing = pi/4
        if left_or_right == 1: #right wing
            angle_of_following = leading_theta - pi/2 - angle_spacing
            x = -1*spacing*cos(angle_of_following)
            y = spacing*sin(angle_of_following)
        elif left_or_right == 2:
            angle_of_following = leading_theta + pi/2 + angle_spacing
            x = spacing*cos(angle_of_following)
            y = spacing*sin(angle_of_following)
        else:
            return -99, -99
        if self.pos[0] + x < 0 or self.pos[1] + y < 0:
            return -99,-99
        else: return x,y


    #Use goal coordinates to navigate the follower robot 
    def follower_pos(self, leader, wing_pos, dt = 0.5):
        goal_x, goal_y = self.goal_position(5,leader)
        if goal_x == -99 and goal_y == -99:
            return 
        goal_theta = leader.record_movement[-1][2]
        np_follower_array = np.array(self.pos)
        
        #Error absolute reference frame
        error_frame = np.array([goal_x - self.record_movement[-1][0], goal_y - self.record_movement[-1][1],
                       goal_theta - self.record_movement[-1][2]])
        
        #Error from follower reference frame
        follower_heading_error = lambda x: np.matmul(np.array([[cos(self.record_movement[x][2]), sin(self.record_movement[x][2]), 0],
                                                    [-sin(self.record_movement[x][2]), cos(self.record_movement[x][2]), 0],
                                                    [0, 0, 1]]).transpose(),
                                                    error_frame)
        
        E_t = lambda x: [follower_heading_error[x], following_heading_error[x]*dt, following_heading_error[x]/dt]
            #E(t) is all the errors. Multiply the errors with the PID controller
            #error, integral error, derivative error
            #time based pid
        H_t = np.array[[E_t(0), 0, 0],
                [0, E_t(1), E_t(2)]]
            
        '''
            #######Change it ############
            if len(self.record_movement) < 3:
                E_k = lambda x: [follower_heading_error(-1)[x], 
                            follower_heading_error(-1)[x],
                            follower_heading_error(-1)[x]]
            else:
                E_k = lambda x: [follower_heading_error(-1)[x] - follower_heading_error(-2)[x], 
                            follower_heading_error(-1)[x],
                            follower_heading_error(-1)[x] - 2*follower_heading_error(-2)[x] + follower_heading_error(-3)[x]]
            ##################################################################

            H_k = np.array([[E_k(0)[0], 0, 0],
                    [0, E_k(1)[1], E_k(2)[2]]])'''
        
        #Ki, Kp, Kd
        K_e = np.array([.5,.5,0.1])
        #[[v],[w]]
        self.vel = 0.1 * np.dot(K_e, H_t.transpose())
        return 
        
     

    #Check for collision with obstacles
    def if_collide_with_landmark(self, x, y):
        for index in range(len(self.landmarks)):
            dist_new_x = self.landmarks[index][0] - x
            dist_new_y = self.landmarks[index][1] - y
            # check the robot crosses the landmark position, Avoidance when obstacle is half of detection range
            if((abs(dist_new_y) < self.measurement_range/2) and (abs(dist_new_x) < self.measurement_range/2)):
                self.world_map[int(self.landmarks[index][0])][int(self.landmarks[index][1])] = 5
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
        self.world_map = np.maximum(self.world_map, robot2.world_map)
        


####### END robot class #######


####### Create its own map #######

# This will help two things-
# a) Easy to print out/ graph
# b) Easy to add/store other records. Avoids unlimited records as well.

# Since it's an np matrix, we can simply just add two 'maps' together, quickly.
# the '+' sign is just a quick-hand 'or' operator.




# ob = robot(100, 5, 6)
# print(ob.record_movement[-1][0])