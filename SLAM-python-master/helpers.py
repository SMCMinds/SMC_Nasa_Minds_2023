from robot_class import robot
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# --------
# this helper function displays the world that a robot is in
# it assumes the world is a square grid of some given size
# and that landmarks is a list of landmark positions(an optional argument)
def display_world(world_size, landmarks=None):
    fig = plt.gcf()
    
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
    
    return fig
    
# --------
# this routine makes the robot data
# the data is a list of measurements and movements: [measurements, [dx, dy]]
# collected over a specified number of time steps, N

#######Changes##########
'''We need to make sure to change this. It continually iterates until
all of the landmarks are detected. This is not good for us since we 
have to create landmarks as we go. Also it seems like the goal of the 
rover is to detect all the landmarks.'''
#########################

def make_data(r, landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):

    # check that data has been made
    try:
        check_for_data(world_size, measurement_range, motion_noise, measurement_noise)
    except ValueError:
        print('Error: You must implement the sense function in robot_class.py.')
        return []
    
    complete = False
    
    # r = robot(world_size, measurement_range, motion_noise, measurement_noise)
    #r.detect_landmarks()

    ### runs until all of the landmarks are detected 
    ### I am going to depend on the list of landmarks not num_of_landmarks
    data = []
    for i in landmarks:
        # seen = [False for row in range(num_landmarks)]
    
        ### guess an initial motion ###
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance
          
        ##loops after N times which was predetermined, however, now I will loop it in an animation    
        ##########CHANGE###############
        for k in range(1):
        ################################    
            # collect sensor measurements in a list, Z
            Z = r.sense()

            # # check off all landmarks that were observed 
            # for i in range(len(Z)):
            #     seen[Z[i][0]] = True
    
            # checks if movement goes out of the graph and moves in a direction if it is
            r.move(dx,dy)

            # collect/memorize all sensor and motion data
            data.append([Z, [dx, dy]])


    print(' ')
    print('Landmarks: ', r.landmarks)
    print(r)

    return data


def check_for_data(world_size, measurement_range, motion_noise, measurement_noise):
    # make robot and landmarks
    r = robot(world_size, measurement_range)
    # r.make_landmarks(num_landmarks)
    
    
    # check that sense has been implemented/data has been made
    test_Z = r.sense()
    if(test_Z is None):
        raise ValueError
