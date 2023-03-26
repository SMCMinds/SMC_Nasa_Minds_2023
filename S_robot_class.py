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
        self.grid=[int(self.pos.x),int(self.pos.y)]
        self.max_speed = MAX_SPEED

        self.vel = pygame.Vector2(random.uniform(-MAX_SPEED , MAX_SPEED ),
                                    random.uniform(-MAX_SPEED , MAX_SPEED ))
        self.vel.normalize() * min(self.vel.magnitude(), self.max_speed) 
        self.acc_mag = MAX_ACCELERATION
        self.acc_angle = np.arctan2(self.vel[1], self.vel[0])
        self.mode = "searching"
        self.target_on_board=False
        self.target_grid=[0,0]

        #trail stuff
        self.show_trail=True
        self.trail = pygame.Surface((WIDTH, HEIGHT),pygame.SRCALPHA)
        self.trail.fill(TRANSPARENT)

       
        
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
        self.grid=[int(self.pos.x),int(self.pos.y)]
        
        if self.target_on_board:
            self.return_base(Current_Map)
        else:
            if self.sense_target(Current_Map.targets):
                target_grid=self.grid
                self.target_on_board=True
            elif self.sense_pheromone_foraging(Current_Map.pheromone_foraging):
                #todo
                a=1
            elif self.sense_pheromone_signaling(Current_Map.pheromone_signalings):
                
                self.search_inside_signal(Current_Map)


    
    def move(self,Current_Map):
        self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
        

        # wall collision; right now it just bounces
        if self.pos.x < ROBOT_SIZE or self.pos.x > WIDTH - ROBOT_SIZE:
            self.vel.x *= -1
            self.acc_angle = math.pi - self.acc_angle

        if self.pos.y < ROBOT_SIZE or self.pos.y > HEIGHT - ROBOT_SIZE:
            self.vel.y *= -1
            self.acc_angle *= -1
        
        self.apply_speed()
    

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

    #TODo
    def sense_pheromone_foraging(self,pheromone_foraging):  
        return False

    def get_acceleration(self):
        return (self.acc_mag*math.cos(self.acc_angle),self.acc_mag*math.sin(self.acc_angle))
    
    def apply_speed(self):
        self.vel += self.get_acceleration()
        self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
        self.pos += self.vel

    def sense_target(self, targets):
        for target in targets:
            dist = self.pos.distance_to(target.pos)
            if dist < SENSOR_RADIUS+Target_RADIUS:
                return True
        return False


    #TODo     
    def search_inside_signal(self,Current_Map):  
        #self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
        cloest_signal=self.get_cloest_signal(Current_Map.pheromone_signalings)
        self.acc_angle=np.arctan2((cloest_signal.pos-self.pos)[1],(cloest_signal.pos-self.pos)[0])
        self.apply_speed()

    def avoid_obstacles(self, obstacles):
        for obstacle in obstacles:
            dist = self.pos.distance_to(obstacle)
            if dist < SENSOR_RADIUS+OBSTACLE_RADIUS:
                desired_vel = (self.pos - obstacle).normalize() * self.max_speed
                desired_angle = desired_vel - self.vel
                self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])

    def return_base(self,Current_Map):
        grid_x=self.grid[0]
        grid_y=self.grid[1]
        #if self.target_grid==self.grid:
        if True:
            #Selects a target_grid based on 2 factors:
            #the Foraging Pheromone intensity and the general direction of base camp
            options = [[-1,-1], [0,-1], [1,-1], [-1,0], [1,0], [-1,1], [0,1], [1,1]]
            weights = []
            base_vector = Current_Map.mission_base.pos - self.pos
            np.arctan2(base_vector[1], base_vector[0])
            for option in options:
                weight=Current_Map.pheromone_foraging[option[0]][option[1]]
                print(option[0])
                weights.append(weight)
                #np.arctan2(desired_angle[1], desired_angle[0])
            selected_grid=random.choices(options, weights)[0]
            self.target_grid =  [grid_x+selected_grid[0],grid_y+selected_grid[1]]
            self.acc_angle=np.arctan2((self.target_grid[1]-self.grid[1]),(self.target_grid[0]-self.grid[0]))
        self.move(Current_Map)
        Current_Map.pheromone_foraging[grid_x][grid_y]=min(1,0.3+Current_Map.pheromone_foraging[grid_x][grid_y])
        return
            

        

                
    
        