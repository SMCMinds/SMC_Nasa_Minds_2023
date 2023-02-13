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
    def __init__(self, world_size=100.0, measurement_range=30.0):
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.landmarks = []
        self.distance = 20.0 #The size of the step that the robot takes

    # returns a positive, random float

    def rand(self):
        return random.random() * 2.0 - 1.0

    # --------
    # move: attempts to move robot by dx, dy. If outside world
    #       boundary, then the move does nothing and instead returns failure

    def move(self, dx, dy):

        x = self.x + dx
        y = self.y + dy

        while x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            orientation = random.random() * 2.0 * pi
            dx = cos(orientation) * self.distance
            dy = sin(orientation) * self.distance
        
        self.x = x
        self.y = y



    # --------
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #
    
    # TODO: paste your complete the sense function, here
    # make sure the indentation of the code is correct
    
    ##
    def sense(self):
        ''' This function list the distances of the landmarks within the
        the sensing radius(measurement_range). It is good since it 
        keeps a list of measurements of distance from each landmark during every
        run time
            '''
           
        measurements = []
        
        # TODO: iterate through all of the landmarks in a world
        for index in range(len(self.landmarks)):
            # distance between landmark and rover
            dist_x = self.landmarks[index][0] - self.x
            dist_y = self.landmarks[index][1] - self.y
            # check if landmark is in range
            if(abs(dist_x) < self.measurement_range and abs(dist_y) < self.measurement_range):
                measurements.append([index, dist_x, dist_y])
        # TODO: return the final, complete list of measurements
        return measurements

    # --------
    # make_landmarks:
    # make random landmarks located in the world
    '''def make_landmarks(self, num_landmarks):
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks'''

    # make_landmarks:
    ###############CHANGE#############
    def detect_landmarks(self):
    # use sensor to determine place of obstacle and create the landmark
        self.landmarks
        # planned: appends a list of the placement of landmarks and records them
    
    ##############CHANGE###############
        

    # called when print(robot) is called; prints the robot's location
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)



####### END robot class #######