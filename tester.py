from math import *

leader = [0,0, 2*pi/3]
p1 = [cos(pi/3)-.001,-sin(pi/3), 0]

def lim_angle(angle):
    while angle < 0:
        angle += 2*pi
    while angle > (2*pi):
        angle -= 2*pi
    return angle

dist_x = p1[0] - leader[0] #x
dist_y = p1[1] - leader[1] #y
leading_theta = lim_angle(leader[2])
limit_angle1 = leading_theta - 90
limit_angle2 = leading_theta + 90

if leading_theta < pi/2:
    if dist_x < 0 and dist_y < 0: #Q3
        pos_theta = abs(atan2(dist_y,dist_x))
        if pos_theta > leading_theta:
            print("right")
        else: print("left")
    if dist_x > 0 and dist_y < 0: #Q4
        print('right')
    if dist_x < 0 and dist_y > 0: #Q2
        print('left')
    
elif leading_theta < pi:
    if dist_x > 0 and dist_y > 0: #Q1
        print('right')
    if dist_x < 0 and dist_y > 0: #Q2
        print('robot not behind')
    if dist_x < 0 and dist_y < 0: #Q3
        print('left')
    if dist_x > 0 and dist_y < 0: #Q4
        pos_theta = pi + atan2(dist_y,dist_x)
        if pos_theta > leading_theta:
            print("right")
        else: print("left")
    
elif leading_theta < 3*pi/2:
    if dist_x > 0 and dist_y > 0: #Q1
        pos_theta = pi + atan2(dist_y,dist_x)
        if pos_theta > leading_theta:
            print("right")
        else: print("left")
    if dist_x < 0 and dist_y > 0: #Q2
        print('left')
    if dist_x < 0 and dist_y < 0: #Q3
        print('robot not behind')
    if dist_x > 0 and dist_y < 0:
        print('right')
        
elif leading_theta < 2*pi:
    if dist_x > 0 and dist_y > 0: #Q1
        print('left')
    if dist_x < 0 and dist_y > 0: #Q2
        pos_theta = pi + atan2(dist_y,dist_x)
        if pos_theta > leading_theta:
            print("right")
        else: print("left")
    if dist_x < 0 and dist_y < 0: #Q3
        print('right')
    if dist_x > 0 and dist_y < 0:
        print('robot not behind')
        
        
        
