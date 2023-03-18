import unittest
from robot_class import robot
from math import *

class TestRobot(unittest.TestCase):
    @classmethod                #works with the class rather than the instance of the class
    def setUpClass(cls):        #runs at the very start
        print('setupClass')
        r_list = []
        for i in range(3):
            r_list.append(robot())
        
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
            r_list.append(robot())
        #Does the angle behind work?
        #The detection distance is maxed by the measurement range
        def x_axis_test():
            r_list[0].pos = [50,50, 0] #reference point angle is pi/3
            #r2 is behind r1
            r_list[1].pos = [40, 40, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [40, 60, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}') 
            r_list[1].pos = [21, 50, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [19, 50, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [50, 55, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [40, 40, 0] #directly behind 1 unit
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            
            #r2 is in front of r2
            r_list[1].pos = [55, 53, 0] #slightly above range
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')
            r_list[1].pos = [55, 52, 0] # on edge of range
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')
            
            r_list[1].pos = [55, 47, 0] #below range
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')      
            r_list[1].pos = [55, 48, 0] #lower edge in range
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')     
            
            r_list[1].pos = [81, 50, 0] #outside detection range to the right
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to detect {r_list[1]}')   
            r_list[1].pos = [79, 50, 0] #farthest edge
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} is supposed to detect {r_list[1]}')           
    
        def y_axis_test():
            r_list[0].pos = [50,50, pi/2] #reference point angle is pi/3
            
            r_list[1].pos = [50, 81, 0] #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [50, 79, 0] #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [53, 55, 0] #outside right
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [52, 55, 0] #inside right
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [47, 55, 0] #outside right
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [48, 55, 0] #inside right
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        def q1_test():
            r_list[0].pos = [50,50, pi/4] #reference point angle is pi/3
            
            r_list[1].pos = [51, 54.829, 0] #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [52, 54.829, 0] #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [51.29, 56, 0] #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [51.29, 53, 0] #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        def q2_test():
            r_list[0].pos = [50,50, 3*pi/4] #reference point angle is pi/3
            r_list[1].pos = [51, 54.829, 0] #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [52, 54.829, 0] #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [51.29, 56, 0] #directly above
            self.assertFalse(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')
            r_list[1].pos = [51.29, 53, 0] #directly above
            self.assertTrue(r_list[0].is_behind(r_list[1]), f'robot {r_list[0]} not supposed to be behind robot {r_list[1]}')

        
        
        x_axis_test()
        y_axis_test()
        q1_test()
            

            

    def move(self):
        pass
    def follower_pos(self):
        pass
    
    def goal_position(self):
        pass
    



if __name__ == '__main__':
    unittest.main()