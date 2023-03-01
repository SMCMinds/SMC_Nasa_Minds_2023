from math import *
import random



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
    def __init__(self, world_size=100.0, measurement_range=30.0, world_landmarks = 0): #world_landmarks only for this simulation
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size * random.random()
        self.y = world_size * random.random()
        self.landmarks = [] #landmarks[x][y] seen by rover
        self.world_landmarks = world_landmarks
        self.distance = 1.0 #The size of the step that the robot takes
        self.orientation = 0
        self.record_movement = [[self.x, self.y, self.orientation]] #record_movement[x,y,orientation]

    # returns a positive, random float

    def rand(self):
        return random.random() * 2.0 - 1.0

    # --------
    # move: attempts to move robot by dx, dy. If outside world
    #       boundary, then the move does nothing and instead returns failure

    def move(self):
        random.seed()
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

    def check_if_collide(self, x, y):
        for index in range(len(self.landmarks)):
            
            # dist_x = self.landmarks[index][0] - self.x
            # dist_y = self.landmarks[index][1] - self.y
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
    #Places the landmark in front of the robot
    def detect_landmarks(self):
        orientation = random.random() * 2.0 * pi
        obstacles = self.simulate_sense()
        for i in range(len(obstacles)): 
            if obstacles[i][0] < self.measurement_range or obstacles[i][1] < self.measurement_range: #Left off here
                # obstacle_x = self.x * cos(orientation)
                # obstacle_y = self.y * sin(orientation)
                self.landmarks.append([self.x + obstacles[i][0], self.y + obstacles[i][1]])
            
    ##############CHANGE###############
    
    
    ##########ONLY FOR SIMULATION#############
    def simulate_sense(self):
        measurements = []
        # TODO: iterate through all of the landmarks in a world
        for index in range(len(self.world_landmarks)):
            # distance between landmark and rover
            dist_x = self.world_landmarks[index][0] - self.x
            dist_y = self.world_landmarks[index][1] - self.y
            # check if landmark is in range
            if(abs(dist_x) < self.measurement_range and abs(dist_y) < self.measurement_range):
                measurements.append([dist_x, dist_y])
        # TODO: return the final, complete list of measurements
        return measurements
        




    # called when print(robot) is called; prints the robot's location
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)



####### END robot class #######