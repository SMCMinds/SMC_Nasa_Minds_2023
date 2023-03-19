
from math import *
import random
import numpy as np



#
class robot:

    
    def __init__(self, world_size=100.0, measurement_range=30.0, world_landmarks=0):
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.landmarks = []  # landmarks[x][y] seen by rover
        self.world_landmarks = world_landmarks
        self.distance = 1.0  # The size of the step that the robot takes
        
        self.pos = [world_size * random.random() - 1 ,world_size * random.random()-1, 2 * pi * random.random()] #position [x,y,theta] , Made purely so computer can access easier
        self.vel = np.array([cos(self.pos[2]),sin(self.pos[2])]) # [dx,dy]
        self.record_movement = [[self.pos[0], self.pos[1], self.pos[2]]] #record_movement[x,y,theta]
        self.record_error = [[0,0,0]]
        self.world_map = np.zeros((int(world_size), int(world_size)), int)
        self.behind = None
        self.behind_angle = None
        
    #Limit the angular orientation between 0 < theta < pi
    def lim_angle(self, angle):
        if angle < 0:
            return angle + 2*pi
        elif angle > 2 * pi:
            return angle - 2*pi
        else: 
            return angle
    
    
    def actual_angle(self, x, y, angle):
        if angle > 0: #Q1 or Q4
            if x>0 and y>0:
                return angle
            else:
                return angle + pi
        elif angle < 0:
            if x<0 and y>0:
                return pi - angle
            else:
                return angle + 2 * pi

    def avoidance(self, x,y,angle):
        
        new_x = x + self.vel[0]
        new_y = y + self.vel[1]
        
        if not self.if_collide_with_landmark(new_x, new_y) and new_x > 1.0 and new_x < self.world_size - 1 and new_y > 1.0 and new_y < self.world_size - 1: 
            return [new_x,new_y,angle]

        #Left turn
        left_turn = angle + 0.01
        left_vel = np.array([2*cos(left_turn), 2*sin(left_turn)])
        left_pos = np.array([x,y]) + left_vel
        while self.if_collide_with_landmark(left_pos[0],left_pos[1]) or left_pos[0] < 1.0 or left_pos[0] > self.world_size - 1 or left_pos[1] < 1.0 or left_pos[1] > self.world_size - 1:
            left_turn = self.lim_angle(left_turn)
            left_turn = left_turn + 0.1
            left_vel = np.array([2*cos(left_turn), 2*sin(left_turn)])
            left_pos = np.array([x,y]) + left_vel
            if abs(left_turn) > 2*pi :
                left_turn = self.lim_angle(left_turn)
                left_vel *= 2
            
        #Right Turn
        right_turn = angle - 0.01
        right_vel = np.array([cos(right_turn), sin(right_turn)])
        right_pos = np.array([x,y]) + right_vel
        while self.if_collide_with_landmark(right_pos[0],right_pos[1]) or right_pos[0] < 1.0 or right_pos[0] > self.world_size - 1 or right_pos[1] < 1.0 or right_pos[1] > self.world_size - 1:              
            right_turn = right_turn - 0.1
            right_vel = np.array([cos(right_turn), sin(right_turn)])
            right_pos = np.array([x,y]) + right_vel
            if abs(right_turn) > 2*pi:
                right_turn = self.lim_angle(right_turn)
                right_vel *= 2
            
        left_pos = left_pos.tolist()

        right_pos = right_pos.tolist()
        
        
        if abs(left_turn - angle) <= abs(right_turn - angle):
            self.vel = left_vel
            final =  [left_pos[0], left_pos[1], left_turn]
            return final
        else:
            self.vel = right_vel
            final = [right_pos[0],right_pos[1],right_turn]
            return final
        
         
       
    #Movement Algorithm
    def move(self):
        '''
            Current Heiarchy of Movement
            1) Avoid Collision
            2) if behind another robot
            
            Add the pheromone contribution
            #Check if behind, if so, set the velocity
            # if self.behind:
            #     x, y = self.goal_position(5, self.behind_angle, self.behind)
            #     self.pos[0] = x
            #     self.pos[1] = y
            #     self.pos[2] = self.behind.pos[2]
                
            #             #Safe to the record
            #     self.record_movement.append(self.pos)
                
            #     #Add position to its local map
            #     self.world_map[int(self.pos[0])][int(self.pos[1])] = 100
                
            #     return
            
            #check Simplified
        '''
        
        # if self.behind:
        #     self.follower_pos(self.behind)
        
        angle = self.pos[2]
        new_pos = [0,0,0]
        #Initialize new position
        #np_follower_array = np.array(self.pos)
        new_pos[0] = self.pos[0] +  self.vel[0]
        new_pos[1] = self.pos[1] +  self.vel[1]
        new_pos[2] = self.pos[2]
            
        #Collision Avoidance
        new_pos = self.avoidance(new_pos[0],new_pos[1],new_pos[2])
        
        #Safe to the record
        self.pos = new_pos
        self.record_movement.append(new_pos)
        
        #Add position to its local map
        self.world_map[int(self.pos[0])][int(self.pos[1])] = 100



    def is_behind(self, robot2, detection_angle = pi/3):
        #make detection 
        angle = self.actual_angle(self.pos[0],self.pos[1], atan2(robot2.pos[1]-self.pos[1], robot2.pos[0] - self.pos[0])) 
        follower_angle = self.actual_angle(self.pos[0], self.pos[1], self.pos[2]) 
        
        correct_lower = follower_angle - detection_angle
        correct_upper = follower_angle + detection_angle
        
        dist = sqrt((robot2.pos[1]-self.pos[1])**2 + (robot2.pos[0] - self.pos[0])**2)
        if (angle > correct_lower) and (angle < correct_upper) and (dist < self.measurement_range * 4): #detects at max measurement range
            self.behind = robot2
            self.behind_angle = angle
            return True
        else:
            self.behind = None
            return False
  
    #Determine whether follower will be on the left or right
    def wing_pos(self, leader):
        #Ensure this function is only called when leader is ahead
        #0 for error
        #1 for right
        #2 for left
        dist_x = leader.pos[0] - self.pos[0] #x
        dist_y = leader.pos[1] - self.pos[1] #y
        
        angle = self.actual_angle(self.pos[0],self.pos[1], atan2(robot2.pos[1]-self.pos[1], robot2.pos[0] - self.pos[0])) 
        leading_theta = leader.pos[2] 
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
    def goal_position(self,spacing, angle_spacing, leader):
        
        left_or_right = self.wing_pos(leader)
        leading_theta = leader[2]
        if left_or_right == 1: #right wing
            angle_of_following = leading_theta - pi + angle_spacing
            x = spacing*cos(angle_of_following)
            y = spacing*sin(angle_of_following)
        elif left_or_right == 2:
            angle_of_following = leading_theta - pi - angle_spacing
            x = spacing*cos(angle_of_following)
            y = spacing*sin(angle_of_following)
        else:
            return -99, -99
        if self.pos[0] + x < 0 or self.pos[1] + y < 0:
            return -99,-99
        else: return self.pos[0] + x, self.pos[1] + y

    #Use goal coordinates to navigate the follower robot 
    def follower_pos(self, leader):
        goal_x, goal_y = self.goal_position(5, self.behind_angle, leader)
        if goal_x == -99 and goal_y == -99:
            return 
        goal_theta = leader.pos[2]
        np_follower_array = np.array(self.pos)
        
        #Error absolute reference frame
        error_frame = np.array([goal_x - self.pos[0], goal_y - self.pos[1],
                       goal_theta - self.pos[2]])
        
        #Error from follower reference frame
        follower_error = np.matmul(np.array([[cos(self.pos[2]), sin(self.pos[2]), 0],
                                                    [-sin(self.pos[2]), cos(self.pos[2]), 0],
                                                    [0, 0, 1]]),
                                                    error_frame)
        
        #Make a record of the errors
        self.record_error.append(follower_error.tolist())
        
        E_t = lambda x: [robot.record_error[-1][x], robot.record_error[-1][x]*dt, robot.record_error[-1][x]/dt]
        
        #Kp, Ki, Kd
        #K_e = np.array([3,2.5,0.02])
        K_e = np.array([0.1, 1, 0.001])
        E_t_x = np.dot(E_k(0), K_e)
        E_t_y = np.dot(E_k(1), K_e)
        #K_pid = K_e * E_k
        # H_k = np.array([[E_k[0], 0, 0],
        #         [0, E_k[1], E_k[2]]])
        
        #[[v],[w]]
        self.vel[0] = E_t_x
        self.vel[1] = E_t_y
        
     

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
                    [self.pos[0] + obstacles[i][0], self.pos[1] + obstacles[i][1]])
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
# print(ob.pos[0])