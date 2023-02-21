from robot_class import robot
from math import *
from matplotlib import animation
import random
import seaborn as sns
from pandas import DataFrame
import matplotlib.pyplot as plt
import numpy as np
import keyboard
import time

###########CHANGES NEEDED####################
'''
So, this algorithm depends on predetermined landmarks that are randomly generated in the world. 
Thus, modification is needed. 

Curr: modify code so it does not rely on  N and make the animation functional

1) The make_landmarks function needs to change because it randomly generates in the map


3) make_data needs to be changed since it is the function that moves the robot
and creates the data

4) transform make_data into something that is gathered constantly in this file

5) use a while loop from previous code to track the data and use the data in real time

6) Find out how sensors can be used to find landmarks

6) Graphing can be figured out while working on the while loop since it gets help from
the display function in the helper file

7) Replace the get_pose_landmark which changes the numpy array to python array

'''



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
    # horizontal_coor = round(random.random() * world_size)
    horizontal_coor = 60
    for i in range(round(world_size/5)):
        landmarks.append([horizontal_coor, i*5])
    
    
    return landmarks

# robot parameters
measurement_range = 4.0     # range at which we can sense landmarks
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
i = 0
while i < time_steps:
    print(i)
    for j in range(len(robot_list)):
        robot_list[j].detect_landmarks()
        robot_list[j].move()
        print_positions(robot_list[j].record_movement[-1])

    # if keyboard.is_pressed('escape'):
    #     running = False
    i+=1


# create the plot
fig = plt.figure()
fig, ax = plt.subplots()

# draw the plot
for i in range(len(robot_list)):
    ax.plot([particle[0] for particle in robot_list[i].record_movement], [
                particle[1] for particle in robot_list[i].record_movement], 'go', linestyle="--")
    ax.plot([particle[0] for particle in robot_list[i].landmarks], [
                particle[1] for particle in robot_list[i].landmarks], marker = '*', linestyle="-")


plt.show()

# Display the final world!

# define figure size
# it determines the size of the window that opens
plt.rcParams["figure.figsize"] = (20, 20)


#Do I even need to animate the graph?
'''
def animate(frame_number):
    global ax  # need it to remove old plot

    # print('frame_number:', frame_number)
    
    # get latest pose and all landmarks
    

    
    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    ax.text(pose[frame_number][0], pose[frame_number][1], 'o', ha='center', va='center', color='r', fontsize=30)
    
    # Draw landmarks if they exists
    if(landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for pos in landmarks:
            if(pos != pose[frame_number]):
                ax.text(pos[0], pos[1], 'x', ha='center', va='center', color='purple', fontsize=20)
    
    # Display final result
    plt.show()

    # move all particles
    r.move()

    # after for-loop

    # remove old plot
    #d.set_data([], [])
    fig.remove()

    # create new plot
    # d, = plt.plot([particle.x for particle in pop], [
    #               particle.y for particle in pop], 'go')
    '''

'''
# --- main ---
fig = plt.figure(1)

# using seaborn, set background grid to gray
sns.set_style("dark")

# Plot grid of values
world_grid = np.zeros((world_size+1, world_size+1))

# Set minor axes in between the labels
#gca, gets current axis
ax=plt.gca()
cols = world_size+1
rows = world_size+1

ax.set_xticks([x for x in range(1,cols)],minor=True )
ax.set_yticks([y for y in range(1,rows)],minor=True)

# Plot grid on minor axes in gray (width = 1)
plt.grid(which='minor',ls='-',lw=1, color='white')

# Plot grid on major axes in larger width
plt.grid(which='major',ls='-',lw=2, color='white')

# fig = plt.gcf()
# draw first plot
# d,  = plt.plot([particle.x for particle in pop], [
#                particle.y for particle in pop], 'go')
# anim = animation.FuncAnimation(fig, animate, frames=200, interval=45, repeat=False)

plt.show()'''