import pygame
import random
import numpy as np
from S_constants_globals import *
from S_robot_class import Robot
from S_pheromone_class import Pheromone_Signaling

# Define Obstacle class
class Obstacle:
    def __init__(self,targets,mission_base):
        invalid=True
        while(invalid):
            self.pos = pygame.Vector2(random.uniform(OBSTACLE_RADIUS, WIDTH-OBSTACLE_RADIUS), 
                                  random.uniform(OBSTACLE_RADIUS, HEIGHT-OBSTACLE_RADIUS))
            invalid=False
            if self.pos.distance_to(mission_base.pos)<(MISSION_BASE_RADIUS+OBSTACLE_RADIUS):
                invalid=True
            for target in targets:
                if self.pos.distance_to(target.pos)<(Target_RADIUS+OBSTACLE_RADIUS):
                    invalid=True
            


# Define Target class
class Target:
    def __init__(self,mission_base):
        invalid=True
        while(invalid):
            self.pos = pygame.Vector2(random.uniform(Target_RADIUS, WIDTH-Target_RADIUS), 
                                    random.uniform(Target_RADIUS, HEIGHT-Target_RADIUS))
            invalid=False
            if self.pos.distance_to(mission_base.pos)<(MISSION_BASE_RADIUS+Target_RADIUS):
                invalid=True


# Define Mission_Base class
class Mission_Base:
    def __init__(self):
        self.pos = pygame.Vector2(WIDTH/2,HEIGHT/2)
    
class General_Map:
    def __init__(self):
        self.mission_base=Mission_Base()
        self.robots = [Robot() for i in range(NUM_ROBOTS)]
        self.targets = [Target(self.mission_base) for i in range (NUM_TARGETS)]
        self.obstacles = [Obstacle(self.targets,self.mission_base) for i in range(NUM_OBSTACLES)]

        self.pheromone_signalings = []
       
        self.pheromone_foraging = [[0.1 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

# Display robots obstacles and Target
def draw_map(screen, Current_Map):
    for pheromone in Current_Map.pheromone_signalings:
        pygame.draw.circle(screen, PINK, (int(pheromone.pos.x), int(pheromone.pos.y)), int(pheromone.radius) )
    for obstacle in Current_Map.obstacles:
        pygame.draw.circle(screen, GRAY, (int(obstacle.pos.x), int(obstacle.pos.y)), OBSTACLE_RADIUS)
    for target in Current_Map.targets:
        pygame.draw.circle(screen, GREEN, (int(target.pos.x), int(target.pos.y)), Target_RADIUS)
    pygame.draw.circle(screen, PURPLE, (int(Current_Map.mission_base.pos.x), int(Current_Map.mission_base.pos.y)), MISSION_BASE_RADIUS)
    for robot in Current_Map.robots:
        pygame.draw.circle(screen, BLUE, (int(robot.pos.x), int(robot.pos.y)), ROBOT_SIZE) 

    
 
        
   
     