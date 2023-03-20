import pygame
import random

from S_constants_globals import * 
from S_robot_class import Robot
from S_map_class import Obstacle,Target,General_Map,draw_map
from S_user_interface import mouse_update,draw_UI



def main():
    #ini_map()
    #Main game loop
    running = True
    Current_Map=General_Map()
    screen.fill(WHITE)
    global FPS
    paused=False
    ultra=False
    frame=0
    while running:
        # Handle events
        frame+=1
        
        for event in pygame.event.get():
            #get mouse input
            mouse_command=mouse_update(event)

            if mouse_command =="restart":
                Current_Map=General_Map()
            if mouse_command =="scale_medium":
                scale_map(SCALE_MEDIUM)
                Current_Map=General_Map()
            if mouse_command =="pause":
                paused=not paused
            if mouse_command =="0.25X":
                FPS=60/4
            if mouse_command =="0.5X":
                FPS=60/2
            if mouse_command =="1X":
                FPS=60
            if mouse_command =="2X":
                FPS=60*2
            if mouse_command =="3X":
                FPS=60*3
            if mouse_command =="ultra":
                FPS=60*15
                if ultra:
                    FPS=60
                ultra=not ultra

            if event.type == pygame.QUIT:
                running = False
                
        # Update robots
        if not paused:
            for robot in Current_Map.robots:
                robot.update(Current_Map)



        if frame>=FPS/60:
            frame=0
            pygame.draw.rect(screen,WHITE,(0,0,WIDTH, SCREEN_HEIGHT))
            draw_map(screen, Current_Map)
            #draw_UI suprising takes a lot longer than the draw_map.
            draw_UI(screen, Current_Map, FPS, clock.get_fps(),paused,ultra)
            # Update the screen
            pygame.display.flip()



        # Wait for the next frame
        clock.tick(FPS)






# Code here will only be executed when the file is run as the main program,
if __name__ == "__main__":
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()    
    main()