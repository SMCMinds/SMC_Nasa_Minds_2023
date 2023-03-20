import pygame
from S_constants_globals import * 



def mouse_update(event):
   
    if event.type == pygame.MOUSEBUTTONUP:
        mouse_x=pygame.mouse.get_pos()[0]
        mouse_y=pygame.mouse.get_pos()[1]
        #restart, pause button
        if (event.button == 1 and mouse_x > WIDTH + 80 and mouse_x < WIDTH + 80 +40 and mouse_y < 28):
            return "restart"
        if (event.button == 1 and mouse_x > WIDTH + 210 and mouse_x < WIDTH + 210 +40 and mouse_y < 28):
            return "pause"
        #speed change
        if (event.button == 1 and mouse_x > WIDTH + 70 and mouse_x < WIDTH + 70 + 61
            and mouse_y > 35 and mouse_y < 35 + 28):
            return "0.25X"
        if (event.button == 1 and mouse_x > WIDTH + 131.5 and mouse_x < WIDTH + 131.5 + 61 
            and mouse_y > 35 and mouse_y < 35 + 28):
            return "0.5X"
        if (event.button == 1 and mouse_x > WIDTH + 193 and mouse_x < WIDTH + 193 + 61
            and mouse_y > 35 and mouse_y < 35 + 28):
            return "1X"
        if (event.button == 1 and mouse_x > WIDTH + 254.5 and mouse_x < WIDTH + 254.5 + 61
            and mouse_y > 35 and mouse_y < 35 + 28):
            return "2X"
        if (event.button == 1 and mouse_x > WIDTH + 316 and mouse_x < WIDTH + 316 + 61
            and mouse_y > 35 and mouse_y < 35 + 28):
            return "3X"
        if (event.button == 1 and mouse_x > WIDTH + 70 and mouse_x < WIDTH + 70 + 61
            and mouse_y > 68 and mouse_y < 68 + 28):
            return "ultra"
        

        if (event.button == 1 and mouse_x > WIDTH + 70 and mouse_x < WIDTH + 70 + 61
            and mouse_y > 68 and mouse_y < 68 + 28):
            return "ultra"



    
    
    

def draw_UI(screen,Current_Map, FPS, real_FPS,paused,ultra):
    # Draw the background Color
    pygame.draw.rect(screen,GRAY,(WIDTH,0,SCREEN_WIDTH -WIDTH, SCREEN_HEIGHT))
    font = pygame.font.SysFont(None, 30)
    text = font.render("Restart: ", True, (0, 0, 0))


    
    if ultra:
        pygame.draw.rect(screen, RED, (WIDTH + 70, 68, 61, 28))
        text = font.render("FPS: " + str(int(real_FPS)), True, (0, 0, 0))
        screen.blit(text, (SCREEN_WIDTH-100, SCREEN_HEIGHT-80))
        return

    
    if False:
        pheromone_foraging=pygame.Surface((WIDTH,HEIGHT), pygame.SRCALPHA)
        for i in range (GRID_WIDTH):
            for j in range (GRID_HEIGHT):
                transp=int(128*(Current_Map.pheromone_foraging[i][j]-0.1))
                pygame.draw.rect(pheromone_foraging,(255,255,0,transp),(WIDTH_SCALE*i,HEIGHT_SCALE*j,WIDTH_SCALE,HEIGHT_SCALE))
        screen.blit(pheromone_foraging,(0,0))
        
  
    #Restart Option
    
    screen.blit(text, (WIDTH, 0))
    pygame.draw.rect(screen, RED, (WIDTH + 80, 3, 40, 25))

    #Pause Option
    text = font.render("Pause: ", True, (0, 0, 0))
    screen.blit(text, (WIDTH+130, 0))
    pygame.draw.rect(screen, YELLOW, (WIDTH + 210, 3, 40, 25))
    if paused:
        pygame.draw.circle(screen,BLACK,(WIDTH + 229,  15.5), 8)

    #Speed Selections
    text = font.render("Speed: " , True, (0, 0, 0))
    screen.blit(text, (WIDTH, 40))
    #draw borders
    pygame.draw.rect(screen, BLACK, (WIDTH + 68, 33, 65, 32), 2)
    pygame.draw.rect(screen, BLACK, (WIDTH + 129.5, 33, 65, 32), 2)
    pygame.draw.rect(screen, BLACK, (WIDTH + 191, 33, 65, 32), 2)
    pygame.draw.rect(screen, BLACK, (WIDTH + 252.5, 33, 65, 32), 2)
    pygame.draw.rect(screen, BLACK, (WIDTH + 314, 33, 65, 32), 2)
    pygame.draw.rect(screen, BLACK, (WIDTH + 68, 66, 65, 32), 2)
    # Draw the background Color
    font = pygame.font.SysFont(None, 30)
    if FPS==15:
        pygame.draw.rect(screen, WHITE, (WIDTH + 70, 35, 61, 28))
    if FPS==30:
        pygame.draw.rect(screen, WHITE, (WIDTH + 131.5, 35, 61, 28))
    if FPS==60:
        pygame.draw.rect(screen, WHITE, (WIDTH + 193, 35, 61, 28))
    if FPS==120:
        pygame.draw.rect(screen, WHITE, (WIDTH + 254.5, 35, 61, 28))
    if FPS==180:
        pygame.draw.rect(screen, WHITE, (WIDTH + 316, 35, 61, 28))
    if FPS==1200:
        pygame.draw.rect(screen, RED, (WIDTH + 70, 68, 61, 28))


    text = font.render("0.25X " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+75, 40))

    text = font.render("0.5X " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+142, 40))
    
    text = font.render("1X " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+205, 40))
    
    text = font.render("2X " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+268, 40))
    
    text = font.render("3X " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+328, 40))

    text = font.render("Ultra " , True, (0, 0, 0))
    screen.blit(text, (WIDTH+75, 73))

    #Robot Roles
    signaling_robots=0
    searching_robots=0
    foraging_robots=0
    for robot in Current_Map.robots:
        if robot.mode=="signaling_target":
            signaling_robots+=1
        elif robot.mode=="searching":
            searching_robots+=1
        elif robot.mode=="foraging":
            foraging_robots+=1

    text = font.render("Signaling: " + str(signaling_robots), True, (0, 0, 0))
    screen.blit(text, (WIDTH, HEIGHT/2))
        
    text = font.render("Searching: " + str(searching_robots), True, (0, 0, 0))
    screen.blit(text, (WIDTH, HEIGHT/2+40))

    text = font.render("Foraging: " + str(foraging_robots), True, (0, 0, 0))
    screen.blit(text, (WIDTH, HEIGHT/2+80))


    #print real FPS
    text = font.render("FPS: " + str(int(real_FPS)), True, (0, 0, 0))
    screen.blit(text, (SCREEN_WIDTH-100, SCREEN_HEIGHT-80))