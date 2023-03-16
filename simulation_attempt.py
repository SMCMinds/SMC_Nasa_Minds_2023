from robot_class import robot
from math import *
from array import *
import random
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time

# world parameters
landmarks = []
num_landmarks = 6
running = True
world_size = 100.0    # size of world (square)


# make_landmarks:
# # make random landmarks located in the world
def make_landmarks(num_landmarks):
    #pick random start point
    for i in range(num_landmarks):
        x_coor = round(random.random() * world_size)
        y_coor = round(random.random() * world_size)
        while x_coor > world_size - 20:
            x_coor = round(random.random() * world_size) 
        while y_coor > world_size - 20:
            y_coor = round(random.random() * world_size)
            
        #Horizontal Wall
        if random.random() <= .5:
            for i in range(5):
                landmarks.append([y_coor, x_coor + i*4])
        else:
            for i in range(5):
                landmarks.append([y_coor + i*4, x_coor])
        #########MAKE A CIRCLE OBSTACLE#################
        
    
    #for a wall
    #horizontal_coor = round(random.random() * world_size)
    horizontal_coor = 60
        
    #You can import a maze if you want make it more complex
    
    
    return landmarks

#############################################
#Pheromone Area!

pheromone_map = []
def make_pheromone_area(world_size):
    n = int(world_size)
    return np.zeros((n, n), int)
    
pheromone_map = make_pheromone_area(world_size)
#############################################

# robot parameters
measurement_range = 5.0     # range at which we can sense landmarks


#Initiate robot class
landmarks = make_landmarks(num_landmarks)
num_of_robots = 3
robot_list = []
for i in range(num_of_robots):
    robot_list.append(robot(world_size, measurement_range, landmarks))

time_steps = 100

#Create sets of 2D Array of empty world
#Last World is over all
world = np.zeros((int(world_size), int(world_size)))

#Graph Landmarks, get rid of it after testing period
b = [[particle[0] for particle in landmarks], [particle[1] for particle in landmarks]]
for i in range(len(b[0])):
        world[int(b[0][i])][int(b[1][i])]=50


# Optimize format of multiple graphs
num_of_graphs = num_of_robots + 1
vertical_graph_stack = int(sqrt(num_of_graphs))
horizontal_graph_stack = vertical_graph_stack
graph_spaces = vertical_graph_stack*horizontal_graph_stack
while num_of_graphs > graph_spaces:
    horizontal_graph_stack += 1
    graph_spaces = vertical_graph_stack*horizontal_graph_stack
    
# create the plot layout
fig, axes = plt.subplots(vertical_graph_stack,horizontal_graph_stack)


################### Neighbour-search ##########################

def neighbour(robot1, robot2, area_size):

    a = robot1.pos[0]
    b = robot1.pos[1]
    c = robot2.pos[0]
    d = robot2.pos[1]

    if ((a < (c + area_size)) and (a > (c - area_size))) and (
    (b < (d + area_size)) and (b > (d - area_size))):
        return True
    return False

#############################################



def animate(data):
    
    world = np.zeros((int(world_size), int(world_size)))
    for index in range(len(robot_list)):
        
        #Share Map when close and check if it is behind another
        for index2 in range(len(robot_list)):
            if index != index2:
                robot_list[index].is_behind(robot_list[index2], pi/3)
                if (neighbour(robot_list[index], robot_list[index2], 20)):
                    robot_list[index].add_records(robot_list[index2])
                    robot_list[index2].add_records(robot_list[index])
        
        empty_world = np.zeros((int(world_size), int(world_size)))
        robot_list[index].detect_landmarks()
        robot_list[index].move()
        
        empty_world = robot_list[index].world_map
        
        fig.axes[index].clear()
        #If want different colormap, add cmap in this parameters
        fig.axes[index].matshow(empty_world)

        world = np.maximum(world, empty_world)
            
    fig.axes[-2].clear()
    fig.axes[-2].matshow(world)


#Initialize the graphs
for ax in axes.flat:
    #If want different colormap, add cmap in this parameters
    im = ax.imshow(world, vmin=0, vmax=100)

#Set Color map
# fig.colorbar(im,ax=axes.ravel().tolist())
fig.colorbar(im,ax=axes.ravel().tolist())

ani = animation.FuncAnimation(fig, animate, frames = 500, interval=1)
# ani.save('orbita.gif', writer='Pillow', fps=30)
plt.show()




#prints all the coordinates that the robot makes
def print_positions(poses):
    print('\n')
    print('Estimated Poses:')
    print('['+ str(poses[0]) + ', ' + str(poses[1]) + ']')
    print('\n')
    i+=1
