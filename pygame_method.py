# import pygame module in this program 
import pygame
import pygame_movement_func
  
# activate the pygame library .  
# initiate pygame and give permission  
# to use pygame's functionality.  
pygame.init()
  
# create the display surface object  
# of specific dimension..e(500, 500).  
win = pygame.display.set_mode((500, 500))
  
# set the pygame window name 
pygame.display.set_caption("Moving rectangle")
  
# object current co-ordinates 
x = 200
y = 200
x2 = 100
y2 = 100
  
# dimensions of the object 
width = 20
height = 20
  
# velocity / speed of movement
vel = 10
  
# Indicates pygame is running
run = True

#object number
a = 1

  
# infinite loop 
while run:
    # creates time delay of 10ms 
    pygame.time.delay(10)
    
    #choose the object that you want to move
    keys = pygame.key.get_pressed()
    if keys[pygame.K_1]:
        a = 1
    elif keys[pygame.K_2]:
        a = 2   
   
    # iterate over the list of Event objects  
    # that was returned by pygame.event.get() method.  
    for event in pygame.event.get():
          
        # if event object type is QUIT  
        # then quitting the pygame  
        # and program both.  
        if event.type == pygame.QUIT:
            # it will make exit the while loop 
            run = False
    
    if a == 1:
        x,y = pygame_movement_func.move(x,y)
    if a == 2:
        x2,y2 = pygame_movement_func.move(x2,y2)          
    # completely fill the surface object  
    # with black colour  
    win.fill((0, 0, 0))
      
    # drawing object on screen which is rectangle here 
    pygame.draw.rect(win, (255, 0, 0), (x, y, width, height))
    pygame.draw.rect(win, (255, 0, 0), (x2, y2, width, height))

    # it refreshes the window
    pygame.display.update() 
  
# closes the pygame window 
pygame.quit()

