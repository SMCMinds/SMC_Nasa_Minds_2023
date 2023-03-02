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


world = []
#2D Array of empty world
for i in range(int(world_size)):
    world.append([])
    for j in range(int(world_size)):
        world[i].append(0)



# create the plot
fig, ax = plt.subplots()
# ax[0].plot(#put all the landmarks and combine the rovers)


#get graph color
def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

# rect = plt.Rectangle((i, -0.5), 1, 1, facecolor=cmap(i))

#From StackOverflow example

def positions():
    a = [[particle[0] for particle in robot_list[index].record_movement], [
        particle[1] for particle in robot_list[index].record_movement]]
    for i in range(len(a[0])):
            world[int(a[0][i])][int(a[1][i])]=100
    b = [[particle[0] for particle in robot_list[index].landmarks], [
                particle[1] for particle in robot_list[index].landmarks]]
    for i in range(len(b[0])):
            world[int(b[0][i])][int(b[1][i])]=50
    return world

def animate(data):
    
    print(data)
    for index in range(len(robot_list)):
        robot_list[index].detect_landmarks()
        robot_list[index].move()
        
        a = [[particle[0] for particle in robot_list[index].record_movement], [
            particle[1] for particle in robot_list[index].record_movement]]
        for i in range(len(a[0])):
                world[int(a[0][i])][int(a[1][i])]=100
        b = [[particle[0] for particle in robot_list[index].landmarks], [
                    particle[1] for particle in robot_list[index].landmarks]]
        for i in range(len(b[0])):
                world[int(b[0][i])][int(b[1][i])]=50
            
        ax.clear()
        ax.matshow(world)
    # return world
        #ax[0,i].set_data(b, color = 'black')

"""
def animate(i): #i[0, ..., world_size]
    ax.clear()
    ax.colorbar(ax.matshow(world))
"""

# def data_gen():
#     while True:
#         yield positions()


# mat = ax[0,0].matshow(world)




# for index in range(len(robot_list)):
#     a = [[particle[0] for particle in robot_list[index].record_movement], [
#         particle[1] for particle in robot_list[index].record_movement]]
#     for i in range(len(a[0])):
#             world[int(a[0][i])][int(a[1][i])]=100
#     b = [[particle[0] for particle in robot_list[index].landmarks], [
#                 particle[1] for particle in robot_list[index].landmarks]]
#     for i in range(len(b[0])):
#             world[int(b[0][i])][int(b[1][i])]=50
plt.colorbar(ax.matshow(world))

# mat2 = ax[1,1].matshow(world)
# mat3 = ax[1,2].matshow(world)
ani = animation.FuncAnimation(fig, animate, frames = 100, interval=20)
plt.show()

# draw the plot



# plt.show()

