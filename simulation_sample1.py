from math import *
from array import *
import random
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time

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
    
    def position(arr, element):
        index = np.where(arr == element)
        return (index[0][0], index[1][0])

    # When robots are disabled, just remove robots from the array
    def disabled():



    
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

# world parameters
landmarks = []
num_landmarks = 6
running = True
world_size = 100.0    # size of world (square)

# make_landmarks:
# # make random landmarks located in the world
def make_landmarks(num_landmarks):
    #for random obstacles
    # for i in range(num_landmarks):
    #     landmarks.append([round(random.random() * world_size),round(random.random() * world_size)])
    
    #for a wall
    #horizontal_coor = round(random.random() * world_size)
    horizontal_coor = 60
    for i in range(round(world_size/5)):
        landmarks.append([horizontal_coor, i*5])
        
    #You can import a maze if you want make it more complex
    
    
    return landmarks


# robot parameters
measurement_range = 5.0     # range at which we can sense landmarks
# distance by which robot (intends to) move each iteratation
distance = 1.0

#Initiate robot class
landmarks = make_landmarks(num_landmarks)
num_of_robots = 3
robot_list = []
for i in range(num_of_robots):
    robot_list.append(robot(world_size, measurement_range, landmarks))

max_steps = 100

#prints all the coordinates that the robot makes
def print_positions(poses):
    print('\n')
    print('Estimated Poses:')
    print('['+ str(poses[0]) + ', ' + str(poses[1]) + ']')
    print('\n')
    # print('Estimated Landmarks:')
    # for i in range(len(landmarks)):
    #     print('['+', '.join('%.3f' % l for l in landmarks[i])+']')
# i = 0
# while i < time_steps:
#     # print(i)
#     for j in range(len(robot_list)):
#         robot_list[j].detect_landmarks()
#         robot_list[j].move()
        # print_positions(robot_list[j].record_movement[-1])

    # if keyboard.is_pressed('escape'):
    #     running = False
    i+=1



# Start here
# Displaying pheromone_world1 map 

#2D Array of empty world
world = np.zeros((int(world_size), int(world_size))) #Position world

#for where the robot has traveled
pheromone_world1 = np.zeros((int(world_size), int(world_size)))

max_pheromone = 5
pheromone = 1
for i in robot_list:
    current_pos = robot.record_movement[-1]
    pheromone_world1[current_pos[0]][current_pos[1]] += 1
    if robot.disabled():
        continue
    # Check if robot is leaving a trace
    if robot:    # Ask
        # Add pheromone value to current cell
        pheromone_world1[current_pos] += pheromone
    # Check if robot should avoid high pheromone value cells
    if pheromone_world1[current_pos[0], current_pos[1]] > max_pheromone:
        # Calculate available moves that avoid high pheromone value cells
        available_moves = [move for move in robot.get_moves() 
                           if pheromone_world1[current_pos[0] + move[0], current_pos[1] + move[1]] <= max_pheromone]
        # Choose a random move from available moves
        if : 
            move = random.choice(available_moves)
            robot.move(move)
            pheromone_world1[current_pos[0] + move[0], current_pos[1] + move[1]] += pheromone
        else:
            # If no available moves, stay in current cell
            robot.move((0,0))
    else:
        # Choose a random move
        move = random.choice(robot.get_moves())
        robot.move(move)
        pheromone_world1[current_pos[0] + move[0], current_pos[1] + move[1]] += pheromone

 
print(pheromone_world1)




#for where they want the excavation robot (when the detect certain pheromone)
pheromone_world2 = np.zeros((int(world_size), int(world_size)))
if 


#for emergency/ if robot is disabled (rovers calling for help)
pheromone_world3 = np.zeros((int(world_size), int(world_size)))
#robot is disabled based on its consecutive movements being in the same position
max_disabled_moves = 3
# Create the pheromone world
pheromone_world3 = np.zeros((int(world_size), int(world_size)))

# Loop through each time step
for t in range(max_steps):
    # Move the robots
    for robot1 in robot:
        robot1.move()
        # Update the pheromone world with the robot's new position
        pheromone_world1[int(robot1.current_pos[0]), int(robot1.current_pos[1])] += 1

    # Detect disabled robots
    disabled_robots = []
    for robot1 in robot:
        # Check if the robot has moved in the last step
        if robot.current_pos != robot.previous_pos:
            # Robot has moved, reset disabled counter
            robot.disabled_counter = 0
        else:
            # Robot has not moved, increment disabled counter
            robot.disabled_counter += 1
            # Check if robot has been disabled for too long
            if robot.disabled_counter >=_max_disabled_moves:
                disabled_robots.append(robot)

    # Mark the traces of disabled robots in the pheromone world
    for robot in disabled_robots:
        pheromone_world3[int(robot.current_pos[0]), int(robot.current_pos[1])] = 'D'

    # Display the pheromone_world3 map
    print(pheromone_world3)



#if robot doesn't detect anything
pheromone_world4 = np.zeros((int(world_size), int(world_size)))
#initial position of the robot
robot_position = [0,0]

for step in range(max_steps):
    # Generating a random movement vector
    move_vector = np.random.randint(-distance, distance+1, size=2)
    
    # Updating the robot's position
    new_position = np.clip(robot_position + move_vector, 0, world_size-1)
    
    # Check if the robot has detected any pheromone
    if pheromone_world4[new_position[0], new_position[1]] > 0:
        print("Robot detected pheromone at position", new_pos)
        break
    
    # If the robot hasn't detected any pheromone, update the pheromone matrix and move the robot
    move(robot_position)
    robot_position = new_position
    
# If the robot hasn't detected any pheromone after the maximum number of steps, move it back to the initial position
else:
    robot_position = [0,0]












# create the plot layout
####TASK######
#Make the number of plots dynamic to the number of robots
##############
fig, axes = plt.subplots(2,2)


#get graph color
def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)



def animate(data):
    
    for index in range(len(robot_list)):
        empty_world = np.zeros((int(world_size), int(world_size)))
        robot_list[index].detect_landmarks()
        robot_list[index].move()
        
        a = [[particle[0] for particle in robot_list[index].record_movement], [
            particle[1] for particle in robot_list[index].record_movement]]
        for i in range(len(a[0])):
                world[int(a[0][i])][int(a[1][i])]=100
                empty_world[int(a[0][i])][int(a[1][i])]=100
        b = [[particle[0] for particle in robot_list[index].landmarks], [
                    particle[1] for particle in robot_list[index].landmarks]]
        for i in range(len(b[0])):
                world[int(b[0][i])][int(b[1][i])]=50
                empty_world[int(b[0][i])][int(b[1][i])]=50
        
        fig.axes[index].clear()
        fig.axes[index].matshow(empty_world)
            
    fig.axes[-2].clear()
    fig.axes[-2].matshow(world)



for ax in axes.flat:
    im = ax.matshow(world, vmin=0, vmax=100)



fig.colorbar(im,ax=axes.ravel().tolist())

ani = animation.FuncAnimation(fig, animate, frames = 500, interval=1)
ani.save('orbita.gif', writer='imagemagick', fps=30)
plt.show()
Footer
© 2023 GitHub, Inc.
Footer navigation
Terms
Privacy
Security
Status
Docs
Contact GitHub
Pricing
API
Training
Blog
About
