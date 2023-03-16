from math import *
import numpy as np
from robot_class import robot


np_follower_array = np.array([1,1,1])
transform = np.array([[2, 0], 
                        [2, 0],
                        [0, 1]])
vel = np.array([4,3])
r1 = robot(100, 5, [[50,50]])
vel_not = None

###Put new_pos in move function because it is moving now ####
### Make in terms of event frame###
vel[1] += 1
new_pos = np.matmul(transform, vel)
# print(new_pos.tolist())
# print(np_follower_array[1])
if vel_not:
    
    print("hi")
else:
    print('no')