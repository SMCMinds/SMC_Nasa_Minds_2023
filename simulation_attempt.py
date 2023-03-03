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

time_steps = 100

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


#2D Array of empty world
world = np.zeros((int(world_size), int(world_size)))



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

