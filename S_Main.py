import pygame
import random
import time

from S_constants_globals import * 
from S_robot_class import Robot
from S_map_class import Obstacle,Target,General_Map,draw_map
from S_user_interface import mouse_update,draw_UI

#Changes most in this file, but also added 3 attributes in the robot class that are trail related
#Click on the robots to show/hide path
#easier to do when paused...
#if multiple robots are at the same location, it toggales all of them on/off
#if you want to fix this, just make it select a random robot from those that are in range of click around line 70
#program seems to run a bit slower, but "ultra" option is still really fast
def screen_shot(screen, Current_Map):
    font = pygame.font.SysFont(None, 30)
    text = font.render("Area Covered: " + str(round(Current_Map.area(),2)) + '%', True, (0, 0, 0))
    screen.blit(text, (WIDTH-250, 40))
    
    time_taken = time.asctime(time.localtime(time.time()))
    time_taken = time_taken.replace(" ", "_")
    time_taken = time_taken.replace(":", ".")
    save_file = 'screenshots/Phero_with_hugging/' + time_taken + '.png'
    rect = pygame.Rect((0,0), (HEIGHT, WIDTH))
    sub = screen.subsurface(rect)
    pygame.image.save(sub, save_file)



def main():
    #ini_map()
    #Main game loop
    running = True
    times = 0
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(TRANSPARENT)

    clock = pygame.time.Clock()   
    Current_Map=General_Map()
    
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
            
            # if mouse_command == "pheromone":
            #     paused=not paused


            if (event.type == pygame.MOUSEBUTTONUP and event.button == 1):
                mouse_x=pygame.mouse.get_pos()[0]
                mouse_y=pygame.mouse.get_pos()[1]
                for robot in Current_Map.robots:
                    if (mouse_x > robot.pos.x - ROBOT_SIZE and mouse_x < robot.pos.x + ROBOT_SIZE
                        and  mouse_y > robot.pos.y - ROBOT_SIZE and mouse_y < robot.pos.y + ROBOT_SIZE):
                        robot.show_trail= not robot.show_trail
            
            if event.type == pygame.QUIT:
                #print(f' {Current_Map.area()}% of the map has been covered')
                screen_shot(screen, Current_Map)
                
                running = False
        if times > 1500:
            screen_shot(screen, Current_Map)
            print(Current_Map.area())
            running = False
        times+=1

        # Update robots
        if not paused:
            for robot in Current_Map.robots:
                #TRAILS HERE
                start_pos= (robot.pos.x, robot.pos.y)
                robot.phero(screen, Current_Map)
                robot.update(Current_Map)
                #pygame.draw.line(robot.trail, RED, start_pos, (robot.pos.x, robot.pos.y), 1)
                
                pygame.draw.circle(robot.trail, TRANSPARENT_GREEN, (robot.pos.x, robot.pos.y), SENSOR_RADIUS/2)

                
                



        if frame>=FPS/60:
            frame=0
            pygame.draw.rect(screen,WHITE,(0,0,WIDTH, SCREEN_HEIGHT))
            draw_map(screen, Current_Map)

            #TRAILS HERE
            for robot in Current_Map.robots:
                if robot.show_trail:
                    screen.blit(robot.trail, (0, 0))
            
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
    for i in range(10): 
        main()
        


