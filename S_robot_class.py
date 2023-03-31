import pygame
import random
import math
import numpy as np
from S_constants_globals import *
from S_pheromone_class import Pheromone_Signaling

# Define Robot class
class Robot:

    def __init__(self):
        self.pos = pygame.Vector2(random.uniform(WIDTH/2 - ROBOT_SIZE, WIDTH/2 + ROBOT_SIZE), 
                                  random.uniform(HEIGHT/2 -  ROBOT_SIZE, HEIGHT/2-ROBOT_SIZE))
        self.grid=[int(self.pos.x),int(self.pos.y)]
        self.target_grid=[0,0]
        self.max_speed = MAX_SPEED
        self.normal_speed = MAX_SPEED/2

        self.vel = pygame.Vector2(random.uniform(-MAX_SPEED , MAX_SPEED ),
                                    random.uniform(-MAX_SPEED , MAX_SPEED ))
        self.vel.normalize() * min(self.vel.magnitude(), self.max_speed) 
        self.acc_mag = MAX_ACCELERATION 
        self.acc_normal = MAX_ACCELERATION/2
        self.acc_angle = np.arctan2(self.vel[1], self.vel[0])
        self.mode = "searching"
        self.target_on_board=False
        ##Parent###
        self.leader = None
        
        #Children#
        self.left_occupied = None
        self.right_occupied = None

        #trail stuff
        self.show_trail=True
        self.trail = pygame.Surface((WIDTH, HEIGHT),pygame.SRCALPHA)
        self.trail.fill(TRANSPARENT)
        
        #Pheromone
        self.move_counter = -10

        #Obstacles
        self.is_obstacle = None
        

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
            #self.return_base(Current_Map)
            a=1
        else:
            if self.sense_target(Current_Map.targets):
                target_grid=self.grid
                self.target_on_board=True
            elif self.sense_pheromone_foraging(Current_Map.pheromone_foraging):
                #todo
                a=1
            elif self.sense_pheromone_signaling(Current_Map.pheromone_signalings):
                
                self.search_inside_signal(Current_Map)
        
    def lim_angle(self, angle):
        if angle < 0:
            return angle + 2* math.pi
        elif angle > 2 * math.pi:
            return angle - 2* math.pi
        else: 
            return angle

    ##############FORMATION MOVEMENT###################################
    #Checks whether it is behind another robot
    def is_behind(self, leader, detection_angle = math.pi/4):
        #initialize 
        y = leader.pos[1]-self.pos[1]
        x = leader.pos[0] - self.pos[0]
        angle = np.arctan2(leader.pos[1]-self.pos[1], leader.pos[0] - self.pos[0])
        follower_angle = np.arctan2(self.vel.y, self.vel.x)
        
        #angular bounds
        correct_lower = follower_angle - detection_angle
        correct_upper = follower_angle + detection_angle
        within_angles = False
        
        #account for different angular bounds because it is tricky when the angle is close to 2pi
        if (angle >= correct_lower) and (angle <= correct_upper):
            within_angles = True
        eq_angle = angle - 2*math.pi
        if (eq_angle >= correct_lower) and (eq_angle <= correct_upper):
            within_angles = True
        eq_angle = angle + 2*math.pi
        if (eq_angle >= correct_lower) and (eq_angle <= correct_upper):
            within_angles = True
          
        #Ensure robot is within distance  
        dist = self.pos.distance_to(leader.pos)
        
        #Uses the angle and abs distance to test if it is behind another robot
        if within_angles and (dist <= SENSOR_RADIUS): #detects at max measurement range
            # self.behind = leader
            # self.behind_angle = angle
            return True
        else:
            # self.behind = None
            return False
  
  
    #Determine whether follower will be on the left or right
    def wing_pos(self, leader):
        #Ensure this function is only called when leader is ahead
        #1 for right
        #2 for left
        dist_x = self.pos[0] - leader.pos[0] #x
        dist_y = self.pos[1] - leader.pos[1] #y
        
        leading_theta = self.lim_angle(np.arctan2(leader.vel.y, leader.vel.x))
        
        if leading_theta < math.pi/2 or leading_theta > 3*math.pi/2:
            y_boundary = dist_x * np.tan(leading_theta) 
            if y_boundary > dist_y:
                return 1 
            else:
                return 2          
        elif math.pi/2 < leading_theta< 3*math.pi/2:
            y_boundary = dist_x * np.tan(leading_theta)
            if  y_boundary > dist_y:
                return 2
            else:
                return 1

        else:
            return 0

    #Return the goal coordinates                    
    def goal_position(self,spacing, angle_spacing, leader):
         
        if self.leader.right_occupied == self:
            left_or_right = RIGHT
        else:
            left_or_right = LEFT
            
        ##########TESTING ONLY#################
        #left_or_right = self.wing_pos(leader)
        ######################################
        
        leading_theta = np.arctan2(leader.vel.y, leader.vel.x)

        #checks that the robots are not facing each other
        if left_or_right == RIGHT: #right wing
            angle_of_following = leading_theta + math.pi + angle_spacing
            x = spacing*math.cos(angle_of_following)
            y = spacing*math.sin(angle_of_following)
            if leader.pos.x + x < ROBOT_SIZE or leader.pos.x + x> WIDTH - ROBOT_SIZE or leader.pos.y + y < ROBOT_SIZE or leader.pos.y + y > HEIGHT - ROBOT_SIZE:
                return None
            leader.right_occupied = self
            return pygame.Vector2(leader.pos.x + x,leader.pos.y + y)
        
        elif left_or_right == LEFT: #left wing
            angle_of_following = leading_theta + math.pi - angle_spacing
            x = spacing*math.cos(angle_of_following)
            y = spacing*math.sin(angle_of_following)
            if leader.pos.x + x < ROBOT_SIZE or leader.pos.x + x> WIDTH - ROBOT_SIZE or leader.pos.y + y < ROBOT_SIZE or leader.pos.y + y > HEIGHT - ROBOT_SIZE:
                return None
            leader.left_occupied = self
            return pygame.Vector2(leader.pos.x + x,leader.pos.y + y)
                
    def search_formation(self, Current_Map):

        #ensure self does not have a leader
        if self.leader is None and not self.left_occupied and not self.right_occupied:
            #for the robot in the list
            for robot in Current_Map.robots:
                #check if robot in list is in front of self and it has no one following
                
                #FOR STRUCTURES UP TO 3
                if robot != self and self.is_behind(robot) and not self.left_occupied and not self.right_occupied and not robot.leader:
                    
                #     #A left or right decision is created 
                    left_or_right = self.wing_pos(robot)

                    #Leader has not recognized self as follower yet since it needs
                    #to go to the last child of the formation                    
                    self.leader = robot
                    if left_or_right == RIGHT and not self.leader.right_occupied:
                        self.leader.right_occupied = self
                    elif left_or_right == LEFT and not self.leader.left_occupied:
                        self.leader.left_occupied = self
                    else:
                        self.leader = None
                    return
                    
                ###FOR STRUCTURES GREATER THAN 3####
                #They somehow become recognize themselves as each others children
                
                '''if robot != self and self.is_behind(robot):
    
                    #A left or right decision is created 
                    left_or_right = self.wing_pos(robot)
                    #Go to parent
                    tracker = robot
                    while tracker.leader:
                        tracker = tracker.leader
                    if tracker is self:
                        return
          
                    ###Going down child trees####
                    #Right Children
                    right_tracker = tracker
                    left_tracker = tracker
                    
                    while right_tracker.right_occupied:
                        right_tracker = right_tracker.right_occupied
                        
                        #If self is already in the structure, then 
                        if right_tracker is self:
                            return
                    
                    
                    #Left Children
                    while left_tracker.left_occupied:
                        left_tracker = left_tracker.left_occupied
                        
                        if left_tracker is self:
                            return

                        
                    #If it surves the end of the formation, set the last robot as parent
                    if left_or_right == RIGHT:
                        self.leader = right_tracker
                        if self.leader.right_occupied:
                            return
                        self.leader.right_occupied = self
                    elif left_or_right == LEFT:
                        self.leader = left_tracker
                        if self.leader.left_occupied:
                            return
                        self.leader.left_occupied = self
                    return'''
                                               
                              
 
        
    def leader_follower(self, Current_Map):
        #ensure robot don't follow each other
        self.search_formation(Current_Map)                
        #self.adust_followers(Current_Map)
        
        ###Ensure code for breaking off the left and right#####
        
        #######################################################
        if self.leader:
            #Check if it got pulled away
            if self.pos.distance_to(self.leader.pos) > SENSOR_RADIUS * 2:
                self.leader = None
                return

            goal_pos = self.goal_position(SENSOR_RADIUS, math.pi/4, self.leader)
            if goal_pos:
                #use kinematics
                #self.acc_normal
                self.vel = self.leader.vel
                self.pos = self.pos.move_towards(goal_pos, 100)
                return

    ###################################################################                   
    
################# PHEROMONE ALGORITHM ###############################
    #It is working
    #Maybe gridify this
    def drop_pheromone(self, grid , x_pos, y_pos):

            
        x = np.arange(0, len(grid))
        y = np.arange(0, len(grid[0]))
        # numpy_grid = np.array(grid)

        r = SENSOR_RADIUS/2
        
        mask = (x[np.newaxis,:] - x_pos)**2 + (y[:,np.newaxis] - y_pos)**2 < r**2
        grid[mask] += 1

        return grid
    
    
    def exploration_pheromone(self,Current_Map):
        #make pheromones a hard obstacle
        max = 0
        max_x = 0
        max_y = 0
        avg = 0.0
        # a = Current_Map.pheromone_grid_1[int(self.pos.x)][int(self.pos.x)]
        if Current_Map.pheromone_grid_1[int(self.pos.x)][int(self.pos.x)] == 255:
            return
        angle = np.arctan2(self.vel.y,self.vel.x)
        for i in range(int(self.pos.x - 10*math.cos(angle)), int(self.pos.x + 10* math.sin(angle))):
            for j in range(int(self.pos.y + 10*math.cos(angle)), int(self.pos.y+ 10* math.sin(angle))):
                avg +=(255- Current_Map.pheromone_grid_1[i][j])
                if Current_Map.pheromone_grid_1[i][j] > max:
                    max = Current_Map.pheromone_grid_1[i][j]
                    max_x = i
                    max_y = j
                    
        avg /= 25
        
        if avg>40:
            
            # if avg > 200:
            #     return            

            dist = self.pos.distance_to(pygame.Vector2(max_x,max_y))
            if dist < SENSOR_RADIUS:
                dist = pygame.Vector2(max_x,max_y) - self.pos 
                if np.arctan2(dist[1],dist[0]) > np.arctan2(self.vel.y, self.vel.y):
                    self.vel = self.vel.rotate(360-45)
                    self.acc_angle -= math.pi/4
                else:
                    self.vel = self.vel.rotate(45)
                    self.acc_angle += math.pi/4
                
                                 
                # desired_angle = desired_vel - self.vel
                # self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])  
            # self.vel.x *= -1
            # self.acc_angle = math.pi - self.acc_angle
            
            
            
        return

        #detect whether the robot goes in a circle
        #Deacticate when the pheromone values get too high
        
        
    
    #This is more of a follower than a repeller
    def following_pheromone(self,grid,x,y):
        #create a list of the robots movement
        #Use that and add a radius to note the area covered
        #For each robot, in pheromone sensing, overlap the pheromones
                
        #Check the surroundings 
        x_accel = 0
        y_accel = 0
        #find a different way
        r = SENSOR_RADIUS/3
        x_arr = np.arange(0, len(grid))
        y_arr = np.arange(0, len(grid[0]))        
                

        #use mask
        avg = 0.0
        area = 1
        #Return true if the area is within circle
        search_area = (x_arr[np.newaxis,:] - x)**2 + (y_arr[:,np.newaxis] - y)**2 < r**2
        min = 0
        # for i in range(0,360, 30):
        #     total = 0
        #     dist = pygame.Vector2.from_polar((r, i))
        # if avg > 3:
        #     self.acc_angle = i / 180 * math.pi
        #     self.vel += pygame.Vector2(self.acc_normal*math.cos(self.acc_angle),self.acc_normal*math.sin(self.acc_angle))
       
        for i in range(0,360, 30):
            total = 0
            for j in range(0,int(r)):
                dist = pygame.Vector2.from_polar((j, i))
                if int(self.pos.x + dist.x) >= WIDTH or int(self.pos.x + dist.x) < 0:
                    total += 100
                    continue
                elif int(self.pos.y + dist.y) >= HEIGHT or int(self.pos.y + dist.y) < 0:
                    total += 100
                    continue
                else:
                    total += grid[int(self.pos.x + dist.x)][int(self.pos.y + dist.y)]
                area += 1
                if grid[int(self.pos.x)][int(self.pos.y)] < grid[int(self.pos.x + dist.x)][int(self.pos.y + dist.y)]:
                    min = i
            avg = total/area
            
            if avg > 3 and grid[int(self.pos.x)][int(self.pos.y)] > 10:
                self.acc_angle = i / 180 * math.pi
                self.vel += pygame.Vector2(self.acc_normal*math.cos(self.acc_angle),self.acc_normal*math.sin(self.acc_angle))

        
        grid[search_area] += 1
        
   
    def phero(self, screen, Current_Map):
        #Copy array 
        
        
        #self.pheromone_grid_1 = pygame.surfarray.array2d(screen)
        #The lower the number, the darker the color is
        Current_Map.pheromone_grid_1 = pygame.surfarray.array_green(screen)

        # print(screen.get_at((400, 400)))
        # print(screen.get_at((420, 410)))
        # print(screen.get_at((450, 350)))
        #pygame.Surface.unlock(screen)
        a=1
        return
        

#############Obstacle###########################
    def hug_obstacle(self, Current_Map):
        ##If it has a right robot, the dist should be sensor radius
        #If it does not, the way it turns should be closer
        avg = 0
        
        for i in range(int(self.pos.x) - 2, int(self.pos.x) + 3):
            for j in range(int(self.pos.y)-2, int(self.pos.y) + 3):
                if i > WIDTH or j > HEIGHT:
                    continue
                avg += (255 - Current_Map.pheromone_grid_1[i][j])
        avg /= 25
            
            
        dist = (self.pos - self.is_obstacle).normalize()
        
        if avg > 3:
            #self.vel = (self.pos - self.is_obstacle).normalize() * self.max_speed
            # desired_angle = desired_vel - self.vel
            # self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])  
            self.is_obstacle = None
        new_vel = dist.rotate(90)
        self.vel = new_vel * self.max_speed
        self.pos += self.vel
   
################################################ 
   
    
    
    def move(self,Current_Map):
        #Weights
        phero_weight = 0
        leader_weight = 0
        avoidance_weight = 0


        #self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
     
        self.check_if_avoid_obstacle([obstacle.pos for obstacle in Current_Map.obstacles],Current_Map)

        if self.leader is None and self.is_obstacle:
            self.hug_obstacle(Current_Map)
            return
            

        
              # wall collision; right now it just bounces
        if self.pos.x < ROBOT_SIZE or self.pos.x > WIDTH - ROBOT_SIZE:
            if self.pos.x < 0:
                self.pos.x = 10 
                self.vel.x = 1
            elif self.pos.x > WIDTH:
                self.pos.x = WIDTH- 3*ROBOT_SIZE 
                self.vel.x = -1
                

            self.vel.x *= -1
            self.acc_angle = math.pi - self.acc_angle
            self.vel += self.get_acceleration()
            self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
            self.apply_speed()
            return
            #avoidance_weight = 5
            
            

        if self.pos.y < ROBOT_SIZE or self.pos.y > HEIGHT - ROBOT_SIZE:
            if self.pos.y < 0:
                self.pos.y = 10 
                self.vel.y = 1
            elif self.pos.y > WIDTH:
                self.pos.y = WIDTH-3 * ROBOT_SIZE 
                self.vel.y = -1
            self.vel.y *= -1
            self.acc_angle *= -1
            avoidance_weight = 5
            self.vel += self.get_acceleration()
            self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
            self.apply_speed()
            return
      

        #Pheromone
        
        self.move_counter += 1
        #self.drop_pheromone(Current_Map.pheromone_grid,self.pos.x,self.pos.y)
        # if self.leader is None:
            #self.avoid_robots(Current_Map)    
        if self.move_counter > 5:
            #Remove the self.vel here and replace
            self.exploration_pheromone(Current_Map)
            self.move_counter = 0
        
        #Formation Movement
        # self.leader_follower(Current_Map)


        self.apply_speed()

 
            
            
        
        
            
    def avoid_obstacles(self, obstacles):
        for obstacle in obstacles:
            dist = self.pos.distance_to(obstacle)
            if dist < SENSOR_RADIUS+OBSTACLE_RADIUS:
                desired_vel = (self.pos - obstacle).normalize() * self.max_speed
                desired_angle = desired_vel - self.vel
                self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])  
    
    def check_if_avoid_obstacle(self, obstacles, Current_Map):   
        #self.is_obstacle = None
        for obstacle in obstacles:
            dist = self.pos.distance_to(obstacle)
            if dist < SENSOR_RADIUS+OBSTACLE_RADIUS:   
                self.is_obstacle = obstacle
                     
            
    
    
    #At this point, it is easier if they just repel from getting too close
    def avoid_robots(self, Current_Map):
        shortest_dist = 1000
        for robot in Current_Map.robots:
            dist = self.pos.distance_to(robot.pos)
            if (dist < shortest_dist) and (robot is not self.right_occupied) and (robot is not self.left_occupied) and (robot is not self):
                shortest_dist = dist
                selected_robot = robot
        
        #find closest robot that is not a follower
        if (shortest_dist < SENSOR_RADIUS/2) and (selected_robot is not self):
            #if wing_pos
            desired_vel = (self.pos - selected_robot.pos).normalize() * self.max_speed
            desired_angle = desired_vel - self.vel
            self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])   

    
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

        self.vel += self.get_acceleration()
        self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
        self.pos += self.vel



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
            

        

