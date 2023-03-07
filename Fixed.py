
#There were some errors with the "Creating simulation environment" to be fixed.

import pygame
width = 20
height = 20
vel = 10  

def rot_center(image, angle, x, y):    
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(center = image.get_rect(center = (x, y)).center)

    return rotated_image, new_rect  


def move(x,y):
    # stores keys pressed 
    keys = pygame.key.get_pressed()
    
    # choose which object to move
    if keys[pygame.K_1]:
        a = 1
    elif keys[pygame.K_2]:
        a = 2   
        
    # if object 1 is selected
    if a == 1:
        # if left arrow key is pressed
        if keys[pygame.K_LEFT] and x>0:
            # decrement in x co-ordinate
            x -= vel
              
        # if left arrow key is pressed
        if keys[pygame.K_RIGHT] and x<500-width:
            # increment in x co-ordinate
            x += vel
             
        # if left arrow key is pressed   
        if keys[pygame.K_UP] and y>0:
            # decrement in y co-ordinate
            y -= vel
              
        # if left arrow key is pressed   
        if keys[pygame.K_DOWN] and y<500-height:
            # increment in y co-ordinate
            y += vel
    
    # if object 2 is selected
    elif a == 2:
        # if left arrow key is pressed
        if keys[pygame.K_a] and x>0:
            # decrement in x co-ordinate
            x -= vel
              
        # if left arrow key is pressed
        if keys[pygame.K_d] and x<500-width:
            # increment in x co-ordinate
            x += vel
             
        # if left arrow key is pressed   
        if keys[pygame.K_w] and y>0:
            # decrement in y co-ordinate
            y -= vel
              
        # if left arrow key is pressed   
        if keys[pygame.K_s] and y<500-height:
            # increment in y co-ordinate
            y += vel
    
    # return the updated coordinates
    return x, y