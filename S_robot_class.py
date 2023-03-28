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
        self.target_grid=[0,0]

        #trail stuff
        self.show_trail=True
        self.trail = pygame.Surface((WIDTH, HEIGHT),pygame.SRCALPHA)
        self.trail.fill(TRANSPARENT)
        
        self.move_counter = 0

        
        

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
        
        #angle = np.arctan2(dist_y, dist_x)
        leading_theta = self.lim_angle(np.arctan2(leader.vel.y, leader.vel.x))

        
        
        if leading_theta < math.pi/2 or leading_theta > 3*math.pi/2:
            # #dist_x is always positive
            # Y_angle = np.tan(leading_theta)
            y_boundary = dist_x * np.tan(leading_theta) 
            if y_boundary > dist_y:
                return 1 
            else:
                return 2          
        elif math.pi/2 < leading_theta< 3*math.pi/2:
            #dist_x is always negativve
            #Tan of Q3 does not work properly
            y_boundary = dist_x * np.tan(leading_theta)
            if  y_boundary > dist_y:
                return 2
            else:
                return 1
        # elif math.pi < leading_theta < 3*math.pi/2:
        #     y_boundary = dist_x * abs(np.tan(leading_theta))
        #     if  y_boundary > dist_y:
        #         return 2
        #     else:
        #         return 1
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
        if not self.leader:
            #for the robot in the list
            for robot in Current_Map.robots:
                #check if robot in list is in front of self and it has no one following
                if robot != self and self.is_behind(robot) and not self.left_occupied and not self.right_occupied and not robot.leader:
                    
                    #A left or right decision is created 
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
                    
                '''  ###FOR STRUCTURES GREATER THAN 3####
                    #Go to parent
                    tracker = robot
                    while tracker.leader:
                        tracker = tracker.leader
          
                    ###Going down child trees####
                    #Right Children
                    while left_or_right == RIGHT and tracker.right_occupied:
                        tracker = tracker.right_occupied
                        
                        #If self is already in the structure, then 
                        if tracker == self:
                            return
                    
                    #Left Children
                    while left_or_right == LEFT and tracker.left_occupied:
                        tracker = tracker.left_occupied
                        
                        if tracker == self:
                            return
                    
                    #If it surves the end of the formation, set the last robot as parent
                    self.leader = tracker
                    if left_or_right == RIGHT:
                        self.leader.right_occupied = self
                    elif left_or_right == LEFT:
                        self.leader.left_occupied = self
                    return
                        '''                        
                              
    #Fix only if it has a leader and 2 childrn
    #Only for structures greater than 3
    def adust_followers(self,Current_Map):
        ###############################################
        ##If the robot is not following anyone, it is able to have left and right occupied
        #Otherwise, it can only have one side occupied
        ############################################

        
       ''' FOR STRUCTURES GREATER THAN 3 
        if self.leader:
            if self.right_occupied and self.left_occupied:
                #If the robot is the left of the leader, transfer all right children 
                tracker = self
                if self.leader.left_occupied == self:
                    #Go to last child
                    while tracker.right_occupied:
                        tracker = tracker.right_occupied
                    
                    #shift all children to left until it reaches a leftpositioned parent
                    while tracker.leader.right_occupied and not self:
                        tracker = tracker.leader
                        tracker.left_occupied = tracker.right_occupied
                        tracker.right_occupied = None
                    
                    # #Finish the final transfer
                    # final_placement = self
                    # while final_placement.left_occupied:
                    #     final_placement = final_placement.left_occupied
                    # final_placement.left_occupied = tracker
                    # tracker.leader = final_placement
                    
                #If the robot is the right of the leader, transfer all left children to the right
                elif self.leader.right_occupied == self:
                    #Go to last child
                    while tracker.left_occupied:
                        tracker = tracker.left_occupied
                    
                    #shift all children to left until it reaches a leftpositioned parent
                    while tracker.leader.left_occupied and not self:
                        tracker = tracker.leader
                        tracker.right_occupied = tracker.left_occupied
                        tracker.left_occupied = None
                    
                    # #Finish the final transfer
                    # final_placement = self
                    # while final_placement.right_occupied:
                    #     final_placement = final_placement.right_occupied
                    # final_placement.right_occupied = tracker
                    # tracker.leader = final_placement
                    
            else:
                return'''
        
        
    def leader_follower(self, Current_Map):
        #ensure robot don't follow each other
        self.search_formation(Current_Map)                
        #self.adust_followers(Current_Map)
        
        ###Ensure code for breaking off the left and right#####
        
        #######################################################
        if self.leader:
            goal_pos = self.goal_position(20, math.pi/4, self.leader)
            if goal_pos:
                #use kinematics
                #self.acc_normal
                self.vel = self.leader.vel
                self.pos = self.pos.move_towards(goal_pos, 100)
                # self.acc_angle=np.arctan2((goal_pos-self.pos)[1],(goal_pos-self.pos)[0])
                # self.vel = pygame.Vector2(0.01 * (goal_pos-self.pos)[1], 0.01 * (goal_pos-self.pos)[0])  
                # self.vel += self.get_acceleration()
                # self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
                #self.pos += self.vel 
                return

    ###################################################################                   
    
################# PHEROMONE ALGORITHM ###############################
    def drop_pheromone(self, grid , x_pos, y_pos):
        x = np.arange(0, len(grid))
        y = np.arange(0, len(grid[0]))
        # numpy_grid = np.array(grid)

        r = SENSOR_RADIUS/2
        
        mask = (x[np.newaxis,:] - x_pos)**2 + (y[:,np.newaxis] - y_pos)**2 < r**2
        grid[mask] += 1
        return grid
    
    def trailing_pheromone(self,grid,x,y):
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

        #use mask
        avg = 0.0
        area = 2 * math.pi * r**2
        #Return true if the area is within circle
        search_area = (x_arr[np.newaxis,:] - x)**2 + (y_arr[:,np.newaxis] - y)**2 < r**2
        min = 100
        for i in range(0,360, 40):
            total = 0
            for j in range(0,int(r)):
                dist = pygame.Vector2.from_polar((r, i))
                total += grid[int(self.pos.x + dist.x)][int(self.pos.y + dist.y)]
        avg = total/area
        
        if avg > 3:
            for i in range(0,360, 40):
                total = 0
                for j in range(r):
                    dist = pygame.Vector2.from_polar((r, i))
                    total += grid[int(self.pos.x + dist.x)][int(self.pos.y + dist.y)]
                    if total < min:
                        min = i
            
            self.acc_angle = i / 180 * math.pi
            self.vel += pygame.Vector2(self.acc_normal*math.cos(self.acc_angle),self.acc_normal*math.sin(self.acc_angle))

        
        grid[search_area] += 1
        
        # length = (WIDTH-1)
        # for i in range(0,length,3):
        #     for j in range(0,length,3):
        #         #create a bunch of polar lines that can calculate the average along each path
        #         if search_area[i][j]:
        #             avg += grid[i][j]
        #             area += 1
        #avg = avg / area           
        # value_area = x[np.newaxis,:] + y[:,np.newaxis] 
        # for i in range(len(search_area)):
        #     for j in range(len(search_area[i])):
        #         if search_area[i][j]:
        #             x_accel -= (avg - grid[i][j]) * (i-x)
        #             y_accel -= (avg - grid[i][j]) * (j-y)
        #total = sum(sum(value_area,[]))  
        
        # #if the robot is in a high intensity area, then increase the velocity. Also, try to find the low value
        # #maybe only activate when the intensity reaches a certain limit
        # a = 0
        # while avg  > 4:
        #     if a < 0:
        #         self.acc_angle += 0.5
        #         self.vel += self.acc_normal*math.cos(self.acc_angle),self.acc_normal*math.sin(self.acc_angle)
        #         a = 4
        #         break
        #     if a == 3:
        #         self.acc_angle -= 0.5
        #     self.vel = self.vel.normalize() * self.vel.magnitude() * avg * 0.5
        #     a-=1
        
        
        # for i in range(len(search_area)):
        #     for j in range(len(search_area[i])):
        #         if search_area[i][j] and grid[i][j] == 0:
        #             desired_vel = (self.pos - pygame.Vector2(i,j)).normalize() * self.max_speed
        #             desired_angle = desired_vel - self.vel
        #             self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])
        #             # self.acc_angle = np.arctan2(j-y, i-x)
        #             self.vel += self.acc_normal*math.cos(self.acc_angle),self.acc_normal*math.sin(self.acc_angle)

        #             self.pos += self.vel
        #             return

            
        #use hypothetical position   
        #new_pos = pygame.Vector2(x,y) + self.vel
        
        
        # # wall collision; right now it just bounces
        # if new_pos.x <= ROBOT_SIZE or new_pos.x >= WIDTH - ROBOT_SIZE:
        #     self.vel.x *= -1
        #     self.acc_angle = math.pi - self.acc_angle
        #     new_pos = pygame.Vector2(x,y) + self.vel
            

        # if new_pos.y <= ROBOT_SIZE or new_pos.y >= HEIGHT - ROBOT_SIZE:
        #     self.vel.y *= -1
        #     self.acc_angle *= -1
        #     new_pos = pygame.Vector2(x,y) + self.vel

        # b = 0
        # c=2
        # while grid[int(new_pos.x)][int(new_pos.y)] > avg:
        #     a = grid[int(new_pos.x)][int(new_pos.y)]
        #     self.vel = self.vel.rotate(30)
        #     new_pos = pygame.Vector2(x,y) + self.vel
        #     b+=1
        #     if b == 10:
                
        #         c+=0.5
        #         self.vel = self.vel.normalize() * self.vel.magnitude() * c
        #         b=0
        #     # wall collision; right now it just bounces
        #     if new_pos.x <= ROBOT_SIZE or new_pos.x >= WIDTH - ROBOT_SIZE:
        #         self.vel.x *= -1
        #         self.acc_angle = math.pi - self.acc_angle
        #         new_pos = pygame.Vector2(x,y) + self.vel
                

        #     if new_pos.y <= ROBOT_SIZE or new_pos.y >= HEIGHT - ROBOT_SIZE:
        #         self.vel.y *= -1
        #         self.acc_angle *= -1
        #         new_pos = pygame.Vector2(x,y) + self.vel
        
        # print(self.vel)
        #need to account for the grid #
        
        
        # #Go towards the place with the least value
        
        # if grid[int(self.pos.x)][int(self.pos.y)] > avg:
        #     self.acc_angle = 0.3
        # #dist = self.pos.distance_to(pygame.Vector2(min_x, min_y))
        # self.acc_angle = np.arctan2(y_accel, x_accel)   
        
        # #However, penalize robots getting too close  
        # #self.avoid_robots(Current_Map)

        # self.vel += self.get_acceleration()
        # # if self.vel.magnitude() < self.max_speed:
        # #     self.vel = -1 * self.vel.normalize() * self.vel.magnitude()
        # # else: 
        # #     self.vel = -1*self.vel.normalize() * self.max_speed
        # self.pos += self.vel
        # return
   
    def phero(self, screen):
        #Copy array 
        self.pheromone_grid_1 = pygame.surfarray.array2d(screen)
        #identify by alpha values
        #enumerate? #Get Values through
        #can I use the mask as a condition in the if statement and pick the closest white target
        search_area = (x_arr[np.newaxis,:] - x)**2 + (y_arr[:,np.newaxis] - y)**2 < r**2
        avg = 0
        length = (WIDTH-1)
        for i in range(0,length,3):
            for j in range(0,length,3):
                if search_area[i][j]:
                    for x in range(i-1,i+2):
                        for y in range(j-1, j+2):
                            #Exclude transparencies that has 255
                            new_screen = screen.convert_alpha()
                            avg += screen.get_alpha()
                    #Search for the minimm average alpha value in the 3x3 spacing
                    #move to the minimum alpha value


        print(screen.get_at((10, 10)))
        #pygame.Surface.unlock(screen)
        a=1
        return
        
    
    
   
   
   
   
    
    
    def move(self,Current_Map):
        
        self.avoid_obstacles([obstacle.pos for obstacle in Current_Map.obstacles])
        self.vel += self.get_acceleration()
        self.vel = self.vel.normalize() * min(self.vel.magnitude(), self.max_speed)
        # if self.leader is None:
        #     self.avoid_robots(Current_Map)
        #     grid_index, x, y = Current_Map.pheromone_grid_func(self)
        #     Current_Map.pheromone_grid[grid_index] = self.drop_pheromone(Current_Map.pheromone_grid[grid_index],x,y)

        #Pheromone
        #If it is not already following, follow pheromone movement
        #Start the after the robot has gone out of the base
        self.move_counter += 1
        if self.leader is None:
            Current_Map.pheromone_grid = self.drop_pheromone(Current_Map.pheromone_grid,self.pos.x,self.pos.y)
            
            if self.move_counter > 100:
                self.trailing_pheromone(Current_Map.pheromone_grid,self.pos.x,self.pos.y)
            
        ''' For Grids
            grid_index, x, y = Current_Map.pheromone_grid_func(self)
            if grid_index != -1:
                
                Current_Map.pheromone_grid[grid_index] = self.drop_pheromone(Current_Map.pheromone_grid[grid_index],x,y)
                if self.move_counter > 100:
                    self.trailing_pheromone(Current_Map.pheromone_grid[grid_index],x,y)
                    self.pos += self.vel'''
            


        self.pos += self.vel
        
        
        #Formation Movement
        self.leader_follower(Current_Map)
        

        # wall collision; right now it just bounces
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
                desired_vel = (self.pos - obstacle).normalize() * self.max_speed
                desired_angle = desired_vel - self.vel
                self.acc_angle = np.arctan2(desired_angle[1], desired_angle[0])        
                
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
            

        

