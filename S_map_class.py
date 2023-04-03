import pygame
import random
import numpy as np
from math import *
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
        
        self.pheromone_grid = np.zeros((WIDTH,HEIGHT))
        self.pheromone_grid_1 = None
        # for i in range(9):
        #     self.pheromone_grid.append(np.zeros((int(WIDTH/3), int(HEIGHT/3))))
        
    def pheromone_grid_func(self, robot):
        #If the robot is in a grid, drop that pheromone in that grid

        if 0 < robot.pos.x <= WIDTH/3 and 0 < robot.pos.y <= HEIGHT/3:
            return 0, robot.pos.x, robot.pos.y
        
        elif WIDTH/3 < robot.pos.x <= 2*WIDTH/3 and 0 < robot.pos.y <= HEIGHT/3:
            return 1, robot.pos.x - WIDTH/3, robot.pos.y
        
        elif 2*WIDTH/3 < robot.pos.x <= WIDTH and 0 < robot.pos.y <= HEIGHT/3:
            return 2, robot.pos.x - 2* WIDTH/3, robot.pos.y
        
        elif 0 < robot.pos.x <= WIDTH/3 and HEIGHT/3 < robot.pos.y <= 2*HEIGHT/3:
            return 3, robot.pos.x, robot.pos.y - HEIGHT/3
        
        elif WIDTH/3 < robot.pos.x <= 2*WIDTH/3 and HEIGHT/3 < robot.pos.y <= 2*HEIGHT/3:
            return 4, robot.pos.x - WIDTH/3, robot.pos.y - HEIGHT/3
        
        elif 2*WIDTH/3 < robot.pos.x <= WIDTH and HEIGHT/3 < robot.pos.y <= 2*HEIGHT/3:
            return 5, robot.pos.x - 2 * WIDTH/3, robot.pos.y - HEIGHT/3
        
        elif 0 < robot.pos.x <= WIDTH/3 and 2 *HEIGHT/3 < robot.pos.y <= HEIGHT:
            return 6, robot.pos.x, robot.pos.y - 2 * HEIGHT/3
        
        elif WIDTH/3 < robot.pos.x <= 2*WIDTH/3 and 2*HEIGHT/3 < robot.pos.y <= HEIGHT:
            return 7, robot.pos.x - WIDTH/3, robot.pos.y - 2 * HEIGHT/3
        
        elif 2*WIDTH/3 < robot.pos.x <= WIDTH and 2*HEIGHT/3 < robot.pos.y <= HEIGHT:
            return 8, robot.pos.x - 2 * WIDTH/3, robot.pos.y - 2 * HEIGHT/3
        
        else:
            print('out of range of pheromone grid')
            return -1,0,0
        
        
                #Border containment for whole grid
        '''        if self.pos.x + SENSOR_RADIUS > WIDTH:
                sensor_width_high = WIDTH - self.pos.x
                sensor_width_low = self.pos.x - SENSOR_RADIUS
            
            elif self.pos.x - SENSOR_RADIUS < 0:
                sensor_width_low = self.pos.x
                sensor_width_high = self.pos.x + SENSOR_RADIUS
                
            else:
                sensor_width_high = self.pos.x + SENSOR_RADIUS
                sensor_width_low = self.pos.x - SENSOR_RADIUS
        
            if self.pos.y + SENSOR_RADIUS > HEIGHT:
                sensor_height_high = HEIGHT - self.pos.y
                sensor_height_low = self.pos.y - SENSOR_RADIUS
            
            elif self.pos.y - SENSOR_RADIUS < 0:
                sensor_height_low = self.pos.y
                sensor_height_high = self.pos.y + SENSOR_RADIUS
                
            else:
            sensor_height_high = self.pos.y + SENSOR_RADIUS
            sensor_height_low = self.pos.y - SENSOR_RADIUS
        '''

    def area(self):
        a = 0
        for i in self.pheromone_grid:
            for j in i:
                if j > 0:
                    a += 1
                        
        #Area of obstacles
        obstacle_area = 0
        for obstacle in self.obstacles:
            obstacle_area = 2*pi* OBSTACLE_RADIUS**2
        total_board = (WIDTH * HEIGHT) - obstacle_area
        return a/total_board * 100
    
    # def area(self):
    #     a = 0
    #     for i in self.pheromone_grid_1:
    #         for j in i:
    #             if j != 255:
    #                 a+=1
                    
    #     obstacle_area = 0
    #     for obstacle in self.obstacles:
    #         obstacle_area += 2*pi* OBSTACLE_RADIUS**2
    #     total_board = (WIDTH * HEIGHT) - obstacle_area
    #     return a/total_board * 100



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

    
 
        
   
     