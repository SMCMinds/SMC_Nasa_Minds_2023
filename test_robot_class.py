import unittest
import numpy as np
from S_robot_class import Robot
from math import *
from S_constants_globals import *
import pygame

class TestRobot(unittest.TestCase):
    @classmethod                #works with the class rather than the instance of the class
    def setUpClass(cls):        #runs at the very start
        print('setupClass')
        
    @classmethod
    def tearDownClass(cls):     #runs at the very end
        print('teardownClass')
    
    def setUp(self):
        print('local setup')
        pass
    
    def tearDown(self):
        pass
    
    def test_is_behind(self):
        r_list = []
        for i in range(3):
            r_list.append(Robot())
        angle = pi/4
        #Does the angle behind work?
        #The detection distance is maxed by the measurement range
        def x_axis_test():
            r_list[0].pos = pygame.Vector2(50,50)  #reference point angle is pi/4
            r_list[0].vel = pygame.Vector2(1,0)
            #r2 is behind r1
            r_list[1].pos = pygame.Vector2(40,40)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(40,60)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}') 
            r_list[1].pos = pygame.Vector2(21,50)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(19,50)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50,55)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(40,40)  
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            
            #r2 is in front of r1
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle)), 50 + (5 * sin(angle)) + 1) #slightly above range
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle)), 50 + (5 * sin(angle)) - 1) # on edge of range
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')
            
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle)), 50 - (5 * sin(angle)) - 1) #below range
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')      
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle)), 50 - (5 * sin(angle)) + 1) #lower edge in range
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')     
            
            r_list[1].pos = pygame.Vector2(50+SENSOR_RADIUS+1, 50) #outside detection range to the right
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')   
            r_list[1].pos = pygame.Vector2(50+SENSOR_RADIUS-1, 50) #farthest edge
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')           
    
        def y_axis_test():
            r_list[0].pos = pygame.Vector2(50,50) #reference point angle is pi/3
            r_list[0].vel = pygame.Vector2(0,1)
            angle_lower = np.arctan2(r_list[0].vel.y, r_list[0].vel.x) - angle
            angle_upper = angle + np.arctan2(r_list[0].vel.y, r_list[0].vel.x) 
            r_list[1].pos = pygame.Vector2(50, 50+SENSOR_RADIUS+1) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50, 50+SENSOR_RADIUS-1) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)) + 2, 50 + (5 * sin(angle_lower))) #outside right
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)) - 1, 50 + (5 * sin(angle_lower))) #inside right
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        def q1_test():
            r_list[0].pos = pygame.Vector2(50,50) #reference point angle is pi/3
            r_list[0].vel = pygame.Vector2(1,1)
            angle_lower = np.arctan2(r_list[0].vel.y, r_list[0].vel.x) - angle
            angle_upper = angle + np.arctan2(r_list[0].vel.y, r_list[0].vel.x)
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)), 50 + (5 * sin(angle_lower))-1) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)), 50 + (5 * sin(angle_lower)) + 1) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) - 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) + 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        def q2_test():
            r_list[0].pos = pygame.Vector2(50,50) #reference point angle is pi/3
            r_list[0].vel = pygame.Vector2(cos(2*pi/3),sin(2*pi/3))
            angle_lower = np.arctan2(r_list[0].vel.y, r_list[0].vel.x) - angle
            angle_upper = angle + np.arctan2(r_list[0].vel.y, r_list[0].vel.x)

            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)) , 50 + (5 * sin(angle_lower))-1) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)), 50 + (5 * sin(angle_lower)) + 1) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) - 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) + 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)), 50 + (5 * sin(angle_upper)) - 1) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)), 50 + (5 * sin(angle_upper)) + 1) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        def q3_test():
            r_list[0].pos = pygame.Vector2(50,50) #reference point angle is pi/3
            r_list[0].vel = pygame.Vector2(cos(7*pi/6),sin(7*pi/6))
            angle_lower = np.arctan2(r_list[0].vel.y, r_list[0].vel.x) - angle
            angle_upper = angle + np.arctan2(r_list[0].vel.y, r_list[0].vel.x)


            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)), 50 + (5 * sin(angle_lower)) + 1) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_lower)), 50 + (5 * sin(angle_lower)) - 1) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) + 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = pygame.Vector2(50 + (5 * cos(angle_upper)) - 1, 50 + (5 * sin(angle_upper))) #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        x_axis_test()
        y_axis_test()
        q1_test()
        q2_test()
        q3_test()
          
          
    def test_wing_pos(self):
        r_list = []
        for i in range(3):
            r_list.append(Robot())
    
        def x_axis_test():
            r_list[0].pos = [50,50,0]
            r_list[0].vel = pygame.Vector2(1,0)
            r_list[1].pos = [40,51,0]
            r_list[1].vel = pygame.Vector2(1,0)
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [55,51,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [40,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [55,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')

        def q1_test():
            r_list[0].pos = [50,50,pi/4]
            r_list[0].vel = pygame.Vector2(cos(pi/4),sin(pi/4))
            r_list[1].vel = pygame.Vector2(1,0)
            
            #q1 
            r_list[1].pos = [50 + 5 * cos(pi/4) - 1,50 + 5 * sin(pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [50 + 5 * cos(pi/4) + 1,50 + 5 * sin(pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            
            #q3
            r_list[1].pos = [50 - 5 * cos(pi/4) - 1,50 - 5 * sin(pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [50 + 5 - cos(pi/4) + 1,50 - 5 * sin(pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')

            #q2, q4

            r_list[1].pos = [40,51,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [60,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')

    
        def q2_test():
            r_list[0].pos = [50,50,2 * pi/3]
            r_list[0].vel = pygame.Vector2(cos(2*pi/3),sin(2*pi/3))
            r_list[1].vel = pygame.Vector2(1,0)
            
            
            #q2
            r_list[1].pos = [50 + 5 * cos(2*pi/3) - 1,50 + 5 * sin(2*pi/3),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [50 + 5 * cos(2*pi/3) + 1,50 + 5 * sin(2*pi/3),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
  
            #q1
            r_list[1].pos = [50 - 5 * cos(2*pi/3) - 1,50 - 5 * sin(2*pi/3),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [50 + 5 - cos(2*pi/3) + 1,50 - 5 * sin(2*pi/3),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')

            #q2, q4

            r_list[1].pos = [40,51,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
            r_list[1].pos = [60,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')

        def q3_test():
            r_list[0].pos = [50,50,7 * pi/6]
            r_list[0].vel = pygame.Vector2(cos(7*pi/6),sin(7*pi/6))
            r_list[1].vel = pygame.Vector2(1,0)
            
            
            #q3
            r_list[1].pos = [50 + 5 * cos(7*pi/6) - 1,50 + 5 * sin(7*pi/6),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [50 + 5 * cos(7*pi/6) + 1,50 + 5 * sin(7*pi/6),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
  
            #q1
            r_list[1].pos = [50 - 5 * cos(7*pi/6) - 1,50 - 5 * sin(7*pi/6),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [50 + 5 - cos(7*pi/6) + 1,50 - 5 * sin(7*pi/6),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')

            #q2, q4

            r_list[1].pos = [40,51,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [60,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
        
        def q4_test():
            r_list[0].pos = [50,50, 7*pi/4]
            r_list[0].vel = pygame.Vector2(cos(7*pi/4),sin(7*pi/4))
            r_list[1].vel = pygame.Vector2(1,0)
            
            
            #q4
            r_list[1].pos = [50 + 5 * cos(7*pi/4) - 1,50 + 5 * sin(7*pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [50 + 5 * cos(7*pi/4) + 1,50 + 5 * sin(7*pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')
  
            #q2
            r_list[1].pos = [50 - 5 * cos(7*pi/4) - 1,50 - 5 * sin(7*pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [50 + 5 - cos(7*pi/4) + 1,50 - 5 * sin(7*pi/4),0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')

            #q1, q3

            r_list[1].pos = [40,49,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 1, f'robot {r_list[1]} is supposed to be on the right side of {r_list[0]}')
            r_list[1].pos = [55,55,0]
            self.assertEqual(r_list[1].wing_pos(r_list[0]), 2, f'robot {r_list[1]} is supposed to be on the left side of {r_list[0]}')

        x_axis_test()
        q1_test()
        q2_test()
        q3_test()
        q4_test()
    
    def test_goal_position(self):
        r_list = []
        for i in range(3):
            r_list.append(Robot())
        spacing_angle = pi/4
        spacing = 5
        r_list[0].pos = pygame.Vector2(50, 50)
        r_list[0].vel = pygame.Vector2(1,0)
        angle = np.arctan2(r_list[0].vel.y, r_list[0].vel.x)
        
        #left
        goal_x_left = 5*cos(angle + pi - spacing_angle) + 50  
        goal_y_left = 5*sin(angle + pi - spacing_angle)  + 50
        
        #right
        goal_x_right = 5*cos(angle + pi + spacing_angle)  + 50
        goal_y_right = 5*sin(angle + pi + spacing_angle)  + 50
        
        r_list[1].pos = pygame.Vector2(55, 55) #front left 
        r_list[1].vel = pygame.Vector2(-1, 0)
        self.assertEqual(r_list[1].goal_position(spacing, spacing_angle, r_list[0]), pygame.Vector2(goal_x_left, goal_y_left), f'{r_list[1]} does not have correct goal')

        r_list[1].pos = pygame.Vector2(55, 35) #front right
        r_list[1].vel = pygame.Vector2(cos(2*pi/3), sin(2*pi/3)) 
        self.assertEqual(r_list[1].goal_position(spacing, spacing_angle, r_list[0]), pygame.Vector2(goal_x_right, goal_y_right), f'{r_list[1]} does not have correct goal')
        
        r_list[1].pos = [35,55, 0]  #back left
        self.assertEqual(r_list[1].goal_position(spacing, spacing_angle, r_list[0]), pygame.Vector2(goal_x_left, goal_y_left), f'{r_list[1]} does not have correct goal')
              
        r_list[1].pos = [45,35, 0]  #back right
        self.assertEqual(r_list[1].goal_position(spacing, spacing_angle, r_list[0]), pygame.Vector2(goal_x_right, goal_y_right), f'{r_list[1]} does not have correct goal')
    
    def test_follower_pos(self):
        pass

    def move(self):
        pass
    
    
    



if __name__ == '__main__':
    unittest.main()
