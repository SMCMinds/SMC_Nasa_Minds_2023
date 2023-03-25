import pygame
import random

from S_constants_globals import * 
from S_robot_class import Robot
from S_map_class import Obstacle,Target,General_Map,draw_map
from S_user_interface import mouse_update,draw_UI

#A quick Branching allowing the drawing of trails.
#Changes: in the Contant Files, a new "TRANSPARENT = (255, 255, 255, 0)"" is defined
#           all other changes are in this Main file
#Note: Line 74 or 75 offers 2 ways to draw the trails. Experiment with commenting out one or the other.

def main():
    #ini_map()
    #Main game loop
    running = True
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(WHITE)

    clock = pygame.time.Clock()   
    Current_Map=General_Map()

    #TRAILS HERE
    trails = pygame.Surface((WIDTH, HEIGHT),pygame.SRCALPHA)
    trails.fill(TRANSPARENT)
    
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
                #TRAILS HERE
                trails = pygame.Surface((WIDTH, HEIGHT))
                trails.fill(WHITE)
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
                #TRAILS HERE
                start_pos= (robot.pos.x, robot.pos.y)
                robot.update(Current_Map)
                pygame.draw.line(trails, RED, start_pos, (robot.pos.x, robot.pos.y), 1)
                #pygame.draw.circle(trails, RED, (robot.pos.x, robot.pos.y), 1)



        if frame>=FPS/60:
            frame=0
            pygame.draw.rect(screen,WHITE,(0,0,WIDTH, SCREEN_HEIGHT))
            draw_map(screen, Current_Map)

            #TRAILS HERE
            screen.blit(trails, (0, 0))
            
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
    main()
