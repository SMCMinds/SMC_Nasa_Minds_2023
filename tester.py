from math import *
import numpy as np
from robot_class import robot

leader = [0,0,pi/3]
def goal_position(spacing, angle_spacing, leader):
        
    angle_of_following = leader[2] - pi + angle_spacing
    x1 = spacing*cos(angle_of_following)
    y1 = spacing*sin(angle_of_following)

    angle_of_following = leader[2] - pi - angle_spacing
    x2 = spacing*cos(angle_of_following)
    y2 = spacing*sin(angle_of_following)

    return [x1,y1], [x2,y2]

print(goal_position(1,pi/6,leader))
    