from math import *
import numpy as np


np_follower_array = np.array([1,1,1])
transform = np.array([[2, 0], 
                        [2, 0],
                        [0, 1]])
vel = np.array([4,3])

###Put new_pos in move function because it is moving now ####
### Make in terms of event frame###
vel[1] += 1
new_pos = np.matmul(transform, vel)
print(new_pos.tolist())
# print(np_follower_array[1])
