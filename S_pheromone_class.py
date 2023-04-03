import pygame
import random
from S_constants_globals import *

class Pheromone_Signaling:
    def __init__(self,pos_x,pos_y):
        self.pos = pygame.Vector2(pos_x, pos_y)
        self.intensity=0
        self.radius=0

