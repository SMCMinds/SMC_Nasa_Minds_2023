import pygame
import random
import math
import numpy as np
from S_constants_globals import *
from S_pheromone_class import Pheromone_Signaling

# Define Robot class
class Robot:
    def __init__(self):
        self.pos = pygame.Vector2(random.uniform(WIDTH/2+ROBOT_SIZE, WIDTH/2-ROBOT_SIZE), 
                                  random.uniform(HEIGHT/2+ROBOT_SIZE, HEIGHT/2-ROBOT_SIZE))
        self.max_speed = MAX_SPEED

        self.vel = pygame.Vector2(random.uniform(-MAX_SPEED , MAX_SPEED ),
                                    random.uniform(-MAX_SPEED , MAX_SPEED ))
        self.vel.normalize() * min(self.vel.magnitude(), self.max_speed) 
        self.acc_mag = MAX_ACCELERATION
        self.acc_angle = np.arctan2(self.vel[1], self.vel[0])
        self.mode = "searching"
        self.target_on_board=False
        self.behind = None

       
        
    def update(self,Current_Map):
        if self.mode =="searching":
            self.update_searching(Current_Map)
        if self.mode =="signaling_target":
            self.spread_pheromone_signaling(Current_Map.pheromone_signalings)
        if self.mode=="foraging":
            self.update_foraging(Current_Map)

    def update_searching(self,Current_Map):
        if self.sense_target(Current_Map.targets):
            self.mode="signaling_target"
        elif self.sense_pheromone_signaling(Current_Map.pheromone_signalings):
            self.mode="foraging"
        else: 
            self.move(Current_Map)

    def update_foraging(self,Current_Map):
        if self.target_on_board:
            a=1
            #return_base(Current_Map)
        else:

            if self.sense_target(Current_Map.targets):

                self.target_on_board=True
            elif self.sense_pheromone_foraging(Current_Map.pheromone_foraging):
                #todo
                a=1
            elif self.sense_pheromone_signaling(Current_Map.pheromone_signalings):
                
                self.search_inside_signal(Current_Map)


        #check if formation can be activated
        if self.behind:
            #if robots see eachother, do nothing
            if self.behind.behind != self:
                self.follower_pos(self.behind)
                
        
        #Initialize new position
        angle = self.pos[2]
        new_pos = [0,0,0]
        new_pos[0] = self.pos[0] +  self.vel[0]
        new_pos[1] = self.pos[1] +  self.vel[1]
        new_pos[2] = self.pos[2]
            
        #Collision Avoidance
        new_pos = self.avoidance(new_pos[0],new_pos[1],new_pos[2])
        
        #Velocity limiter
        while self.vel[0] > 1.5 or self.vel[1] > 1.5:
            self.vel[0] = self.vel[0] / 2
            self.vel[1] = self.vel[1] / 2
        
        #Save to the record
        self.pos = new_pos
        self.record_movement.append(new_pos)
        
        #Add position to its local map
        self.world_map[int(self.pos[0])][int(self.pos[1])] = 10

    def lim_angle(self, angle):
        if angle < 0:
            return angle + 2*pi
        elif angle > 2 * pi:
            return angle - 2*pi
        else: 
            return angle

    #Checks whether it is behind another robot
    def is_behind(self, robot2, detection_angle = pi/4):
        #initialize 
        
        angle = atan2(robot2.pos[1]-self.pos[1], robot2.pos[0] - self.pos[0])
        follower_angle = np.arctan2(self.vel.y, self.vel.x)
        
        #angular bounds
        correct_lower = follower_angle - detection_angle
        correct_upper = follower_angle + detection_angle
        within_angles = False
        
        #account for different angular bounds because it is tricky when the angle is close to 2pi
        if (angle >= correct_lower) and (angle <= correct_upper):
            within_angles = True
        eq_angle = angle - 2*pi
        if (eq_angle >= correct_lower) and (eq_angle <= correct_upper):
            within_angles = True
        eq_angle = angle + 2*pi
        if (eq_angle >= correct_lower) and (eq_angle <= correct_upper):
            within_angles = True
          
        #Ensure robot is within distance  
        dist = self.pos.distance_to(robot2.pos)
        
        #Uses the angle and abs distance to test if it is behind another robot
        if within_angles and (dist <= SENSOR_RADIUS): #detects at max measurement range
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
        
        angle = self.lim_angle(atan2(leader.pos[1]-self.pos[1], leader.pos[0] - self.pos[0])) 
        
        leading_theta = leader.pos[2] 
        
        
        if leading_theta < pi:
            if leading_theta < angle < leading_theta + pi:
                return 1 
            else:
                return 2          
        elif leading_theta < 2*pi:
            if leading_theta - pi < angle < leading_theta:
                return 2
            else:
                return 1
        else:
            return 0

    #Return the goal coordinates                    
    def goal_position(self,spacing, angle_spacing, leader):
        left_or_right = self.wing_pos(leader)
        leading_theta = leader.pos[2]
        
        #checks that the robots are not facing each other
        if abs(leading_theta-self.pos[2]) < pi/2:
            if left_or_right == 1: #right wing
                angle_of_following = leading_theta + pi + angle_spacing
                x = spacing*cos(angle_of_following)
                y = spacing*sin(angle_of_following)
            elif left_or_right == 2: #left wing
                angle_of_following = leading_theta + pi - angle_spacing
                x = spacing*cos(angle_of_following)
                y = spacing*sin(angle_of_following)
        else: #robots facing each other
            return -99, -99
        
        #ensure goal is not outside the map
        if self.pos[0] + x < 0 or self.pos[0] + x > self.world_size or self.pos[1] + y < 0 or self.pos[1] + y > self.world_size:
            return -99,-99
        else: return leader.pos[0] + x, leader.pos[1] + y

    
    def move(self,Current_Map):
        
        #Checking for Formation
        if self.behind:
            #if robots see eachother, do nothing
            if self.behind.behind != self:
                self.vel = self.goal_position(5, math.pi/4, self.behind).move_towards()
                
                
        self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
        self.vel += self.get_acceleration()
        self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
        self.pos += self.vel
        
                
 
        #wall collision; right now it just bounces
        if self.pos.x < ROBOT_SIZE or self.pos.x > WIDTH - ROBOT_SIZE:
            self.vel.x *= -1
            self.acc_angle = math.pi - self.acc_angle

        if self.pos.y < ROBOT_SIZE or self.pos.y > HEIGHT - ROBOT_SIZE:
            self.vel.y *= -1
            self.acc_angle *= -1
    
    def avoid_obstacles(self, obstacles):
        for obstacle in obstacles:
            dist = self.pos.distance_to(obstacle)
            if dist < SENSOR_RADIUS+OBSTACLE_RADIUS:
                angle_to_obstacle = np.arctan2(obstacle.y-self.pos.y, obstacle.x-self.pos.x)
                robot_angle = np.arctan2(self.vel.y, self.vel.x)
                edge_of_obstacle = pygame.Vector2(OBSTACLE_RADIUS*math.cos(angle_to_obstacle), OBSTACLE_RADIUS*math.sin(angle_to_obstacle))
                if angle_to_obstacle - robot_angle > math.pi:
                    pass
                if angle_to_obstacle > robot_angle:
                    self.acc_angle = robot_angle - math.pi/2
                else:
                    self.acc_angle = robot_angle + math.pi/2
                # desired_vel = (self.pos - obstacle).normalize() * self.max_speed
                # desired_angle = desired_vel - self.vel



    
    def spread_pheromone_signaling(self,pheromone_signalings):
        not_inside_pheromone_signaling=True
        for pheromone_signaling in pheromone_signalings:
            dist = self.pos.distance_to(pheromone_signaling.pos)
            if dist < (pheromone_signaling.radius+SENSOR_RADIUS):
                not_inside_pheromone_signaling=False
                
                if MAX_PHEROMONE_INTENSITY<=pheromone_signaling.intensity:
                    self.mode="foraging"
                else :
                    pheromone_signaling.intensity+=1
                    pheromone_signaling.radius+=SPREAD_SPEED
        if not_inside_pheromone_signaling:
            pheromone_signalings.append(Pheromone_Signaling(self.pos.x,self.pos.y))
    
         
    def sense_pheromone_signaling(self,pheromone_signalings):  
        for pheromone_signaling in pheromone_signalings:
            dist = self.pos.distance_to(pheromone_signaling.pos)
            if dist < (pheromone_signaling.radius):
                return True
        return False

    def get_cloest_signal(self,pheromone_signalings): 
        max_dis=WIDTH*2 
        cloest_singal=[]
        for pheromone_signaling in pheromone_signalings:
            dist = self.pos.distance_to(pheromone_signaling.pos)
            if dist <  max_dis:
                max_dis=dist
                cloest_singal=pheromone_signaling
        return cloest_singal

    def sense_pheromone_foraging(self,pheromone_foraging):  
        return False

    def get_acceleration(self):
        return (self.acc_mag*math.cos(self.acc_angle),self.acc_mag*math.sin(self.acc_angle))

    def sense_target(self, targets):
        for target in targets:
            dist = self.pos.distance_to(target.pos)
            if dist < SENSOR_RADIUS+Target_RADIUS:
                return True
        return False


    #potentional loop here        
    def search_inside_signal(self,Current_Map):  
        #self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
        cloest_signal=self.get_cloest_signal(Current_Map.pheromone_signalings)
        self.acc_angle=np.arctan2((cloest_signal.pos-self.pos)[1],(cloest_signal.pos-self.pos)[0])

        self.vel += self.get_acceleration()
        self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
        self.pos += self.vel



    def return_base(self,Current_Map):
        a=1

                
'''
    def avoid_obstacles(self, obstacles):
        for obstacle in obstacles:
            dist = self.pos.distance_to(obstacle)
            if dist < SENSOR_RADIUS+OBSTACLE_RADIUS:
                desired_vel = (self.pos - obstacle).normalize() * self.max_speed
                desired_angle = desired_vel - self.vel
                self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])



'''