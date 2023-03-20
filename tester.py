from math import *
import numpy as np
from robot_class import robot


r1 = robot()
r2 = robot()


r1.pos = np.array([0.0,0.0, 0.0])
r2.pos = np.array([20.0,0.0, 0.0])
vel = [0,0]
#K_e = np.array([3, 2.5, 0.02])
#K_e = np.array([1, 10, 0.01])
K_e = np.array([0.1, 1, 0.001])
def lim_angle(angle):
    while angle < 0:
        angle += 2*pi
    while angle > (2*pi):
        angle -= 2*pi
    return angle
        

def follower_pos(robot, constant, dt = 0.5):
    #leader replaced with goals
    goal_x = 40
    goal_y = 50
    goal_theta = 30
    np_follower_array = np.array(robot.pos)
    
    #Error absolute reference frame
    error_frame = np.array([goal_x - robot.pos[0], goal_y - robot.pos[1],
                    goal_theta - robot.pos[2]])
    
    #Error from follower reference frame
    follower_error = np.matmul(np.array([[cos(robot.pos[2]), sin(robot.pos[2]), 0],
                                                [-sin(robot.pos[2]), cos(robot.pos[2]), 0],
                                                [0, 0, 1]]),
                                                error_frame)
    
    #Make a record of the errors
    robot.record_error.append(follower_error.tolist())
    
    '''#######Change it ############
        if len(robot.record_error) < 3:
            E_k = [robot.record_error[-1][0], robot.record_error[-1][1], robot.record_error[-1][2]]
        else:
            E_k = [robot.record_error[-1][0] - robot.record_error[-2][0], 
                        robot.record_error[-1][1],
                        robot.record_error[-1][2] - 2*robot.record_error[-2][2] + robot.record_error[-3][2]]

        H_k = np.array([[E_k[0], 0, 0],
                [0, E_k[1], E_k[2]]])
        K_pid = K_e * E_k
        ##################################################################

        return 0.1 * np.dot(H_k, K_pid)
        '''

    #Ki,Kp,Kd
    E_t = lambda x: [robot.record_error[-1][x], robot.record_error[-1][x] * dt, robot.record_error[-1][x]/dt]
    
    E_t_x = np.dot(constant, E_t(0))
    E_t_y = np.dot(constant, E_t(1))
    #E_t_theta = np.dot(constant, E_t(2))

    ##################################################################

    return E_t_x, E_t_y
    
    
    
i= 0
while i < 100:
    vel1 = follower_pos(r1, K_e)
    vel2 = follower_pos(r2, K_e)

    r1.pos[0] = r1.pos[0] + vel1[0]
    r1.pos[1] += vel1[1]
    #r1.pos[2] += vel1[2]
    #r1.pos[2] = lim_angle(r1.pos[2])
    r2.pos[0] +=  vel2[0]
    r2.pos[1] += vel2[1]

    #r2.pos[2] = lim_angle(r2.pos[2])
    
    #fix this as much as u can
    print(r1.pos.tolist())
    print(r2.pos.tolist())

    i+=1








# r1.pos = np.array([0,0,0])
# r2.pos = np.array([0,0,0])
# #K_e = np.array([3, 2.5, 0.02])
# #K_e = np.array([1, 10, 0.01])
# K_e = np.array([1, 10, 0.01])
# def lim_angle(angle):
#     while angle < 0:
#         angle += 2*pi
#     while angle > (2*pi):
#         angle -= 2*pi
#     return angle
        

# def follower_pos(robot, constant, dt = 0.5):
#     goal_x = 50
#     goal_y = 50
#     goal_theta = 0
#     np_follower_array = np.array(robot.pos)
    
#     #Error absolute reference frame
#     error_frame = np.array([goal_x - robot.pos[0], goal_y - robot.pos[1],
#                     goal_theta - robot.pos[2]])
    
#     #Error from follower reference frame
#     follower_error = np.matmul(np.array([[cos(robot.pos[2]), sin(robot.pos[2]), 0],
#                                                 [-sin(robot.pos[2]), cos(robot.pos[2]), 0],
#                                                 [0, 0, 1]]),
#                                                 error_frame)
    
#     #Make a record of the errors
#     robot.record_error.append(follower_error.tolist())
    
#     '''#######Change it ############
#         if len(robot.record_error) < 3:
#             E_k = [robot.record_error[-1][0], robot.record_error[-1][1], robot.record_error[-1][2]]
#         else:
#             E_k = [robot.record_error[-1][0] - robot.record_error[-2][0], 
#                         robot.record_error[-1][1],
#                         robot.record_error[-1][2] - 2*robot.record_error[-2][2] + robot.record_error[-3][2]]

#         H_k = np.array([[E_k[0], 0, 0],
#                 [0, E_k[1], E_k[2]]])
#         K_pid = K_e * E_k
#         ##################################################################

#         return 0.1 * np.dot(H_k, K_pid)
#         '''

#     if len(robot.record_error) < 3:
#     E_t = lambda x: [robot.record_error[-1][0], robot.record_error[-1][1], robot.record_error[-1][2]]
#     else:
#         E_k = [robot.record_error[-1][0] - robot.record_error[-2][0], 
#                     robot.record_error[-1][1],
#                     robot.record_error[-1][2] - 2*robot.record_error[-2][2] + robot.record_error[-3][2]]

#     H_k = np.array([[E_k[0], 0, 0],
#             [0, E_k[1], E_k[2]]])
#     K_pid = K_e * E_k
#     ##################################################################

#     return 0.1 * np.dot(H_k, K_pid)
    
    
    
# i= 0
# while i < 100:
#     vel1 = follower_pos(r1, K_e)
#     transform = np.array([[cos(r1.pos[2]), 0], 
#                         [sin(r1.pos[2]), 0],
#                         [0, 1]])
#     r1.pos = r1.pos +  np.matmul(transform, vel1)
#     r1.pos[2] = lim_angle(r1.pos[2])
#     vel2 = follower_pos(r1, K_e)
#     transform = np.array([[cos(r2.pos[2]), 0], 
#                         [sin(r2.pos[2]), 0],
#                         [0, 1]])
#     r2.pos = r2.pos +  np.matmul(transform, vel2)
#     r2.pos[2] = lim_angle(r2.pos[2])

#     i+=1

# print(r1.pos.tolist())
# print(r2.pos.tolist())













#########TEST BEST PID Values##############
# r1 = robot()
# r2 = robot()
# best_r1 = 100
# best_r2 = 100
# best_k = np.array([1,10,0.2])
# # #Ki, Kp, Kd
# for i in range(10):
#     for j in range(50):
#         for k in range(50):
#             r1.pos = np.array([0,0,0])
#             r2.pos = np.array([0,0,0])
#             K_e = np.array([0.03*i, 1 + 0.5 * j, 0.01*k])

#             def lim_angle(angle):
#                 while angle < 0:
#                     angle += 2*pi
#                 while angle > (2*pi):
#                     angle -= 2*pi
#                 return angle
                    

#             def follower_pos(robot, constant, dt = 0.5):
#                 goal_x = 50
#                 goal_y = 50
#                 goal_theta = 0
#                 np_follower_array = np.array(robot.pos)
                
#                 #Error absolute reference frame
#                 error_frame = np.array([goal_x - robot.pos[0], goal_y - robot.pos[1],
#                                 goal_theta - robot.pos[2]])
                
#                 #Error from follower reference frame
#                 follower_error = np.matmul(np.array([[cos(robot.pos[2]), sin(robot.pos[2]), 0],
#                                                             [-sin(robot.pos[2]), cos(robot.pos[2]), 0],
#                                                             [0, 0, 1]]),
#                                                             error_frame)
                
#                 #Make a record of the errors
#                 robot.record_error.append(follower_error.tolist())
                
#                 # #######Change it ############
#                 # if len(record_error) < 3:
#                 #     E_k = [record_error[-1][0], record_error[-1][1], record_error[-1][2]]
#                 # else:
#                 #     E_k = [record_error[-1][0] - record_error[-2][0], 
#                 #                 record_error[-1][1],
#                 #                 record_error[-1][2] - 2*record_error[-2][2] + record_error[-3][2]]

#                 # H_k = np.array([[E_k[0], 0, 0],
#                 #         [0, E_k[1], E_k[2]]])
                
#                 # ##################################################################
                    
#                 E_t = lambda x: [robot.record_error[-1][x], robot.record_error[-1][x]*dt, robot.record_error[-1][x]/dt]

#                 H_t = np.array([[E_t(0)[0], 0, 0],
#                     [0, E_t(1)[1], E_t(2)[2]]])

                

#                 #[[v],[w]]
                
#                 '''
#                     K_e = np.array([1,10,0.1])
#                     E_t_x = np.dot(E_t(0),K_e)
#                     E_t_y = np.dot(E_t(1),K_e)
#                     E_t_theta = np.dot(E_t(2),K_e)

#                     H_t = np.array([[E_t_x, 0, 0],
#                         [0, E_t_y, E_t_theta]])'''

#                 return 0.1 * np.dot(K_e, H_t.transpose())
                
#             i= 0
#             while i < 100:
#                 vel1 = follower_pos(r1, K_e)
#                 transform = np.array([[cos(r1.pos[2]), 0], 
#                                     [sin(r1.pos[2]), 0],
#                                     [0, 1]])
#                 r1.pos = r1.pos +  np.matmul(transform, vel1)
#                 r1.pos[2] = lim_angle(r1.pos[2])
#                 vel2 = follower_pos(r1, K_e)
#                 transform = np.array([[cos(r2.pos[2]), 0], 
#                                     [sin(r2.pos[2]), 0],
#                                     [0, 1]])
                
#                 r2.pos = r2.pos +  np.matmul(transform, vel2)
#                 r1.pos[2] = lim_angle(r1.pos[2])

#                 i+=1
            
#             dist_from_best1 = sqrt((r1.pos[0] - 50)**2 + (r1.pos[1] - 50)**2)
#             dist_from_best2 = sqrt((r2.pos[0] - 50)**2 + (r2.pos[1] - 50)**2)

#             if dist_from_best1 < best_r1 and dist_from_best2 < best_r2:
#                 best_r1 = dist_from_best1 
#                 best_r2 = dist_from_best2
#                 best_k = K_e
#             print(K_e)

        
# print(best_k.tolist())
# print(r1.pos.tolist())
# print(r2.pos.tolist())