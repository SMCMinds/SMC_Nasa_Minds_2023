from math import *
import numpy as np
from robot_class import robot


np_follower_array = np.array([1,1,1])
transform = np.array([[2, 0], 
                        [2, 0],
                        [0, 1]])
vel = np.array([4,3])

# r1 = robot(100, 5, [[50,50]])
vel_not = None

###Put new_pos in move function because it is moving now ####
### Make in terms of event frame###
vel[1] += 1
new_pos = np.matmul(transform, vel)
# print(new_pos.tolist())
# print(np_follower_array[1])
# if vel_not:
    
#     print("hi")
# else:
#     print('no')


#Lambda Test
error = lambda x: np.matmul(transform, vel) 
error_frame = np.array([1, 2,
                3])
follower_heading_error = np.matmul([[cos(1), sin(1), 0],
                                            [-sin(1), cos(1), 0],
                                            [0,0,1]],
                                            error_frame)

#print(follower_heading_error.tolist())

a = float(1.09)
H_k = np.array([[a, 0, 0],
                [0, 2.1 , 1.3]])
        #Ki, Kp, Kd
K_e = np.array([a,10.1221212,1.078969])
        #[[v],[w]]
vel = np.dot(K_e, H_k.transpose())
print(vel)