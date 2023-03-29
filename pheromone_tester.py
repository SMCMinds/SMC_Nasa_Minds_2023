import pygame
import random
import math
import numpy as np

class Pheromone:
    def __init__(self, grid_width, grid_height):
        self.grid = np.zeros((grid_width, grid_height))
        
    def update(self):
        self.grid *= 0.99
        
    def deposit(self, x, y, intensity):
        i = int(x)
        j = int(y)
        if i >= 0 and i < self.grid.shape[0] and j >= 0 and j < self.grid.shape[1]:
            self.grid[i][j] += intensity
            
    def get_intensity(self, x, y):
        i = int(x)
        j = int(y)
        if i >= 0 and i < self.grid.shape[0] and j >= 0 and j < self.grid.shape[1]:
            return self.grid[i][j]
        else:
            return 0.0
        
class Particle:
    def __init__(self, x, y):
        self.position = [x, y]
        self.velocity = [0, 0]
        self.max_speed = 5.0
        self.max_force = 0.1
        self.radius = 10.0
        
    def update(self, pheromones):
        avoidance_force = self.avoid_pheromones(pheromones)
        self.apply_force(avoidance_force)
        self.limit_speed()
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]
        
    def avoid_pheromones(self, pheromones):
        avoidance_force = [0, 0]
        for pheromone in pheromones:
            intensity = pheromone.get_intensity(self.position[0], self.position[1])
            if intensity > 0:
                direction = self.direction_away_from_pheromone(pheromone)
                avoidance_force[0] += direction[0] * intensity
                avoidance_force[1] += direction[1] * intensity
        return avoidance_force
    
    def direction_away_from_pheromone(self, pheromone):
        dx = self.position[0] - int(self.position[0])
        dy = self.position[1] - int(self.position[1])
        if dx > 0.5:
            i1 = int(self.position[0]) + 1
        else:
            i1 = int(self.position[0])
        if dy > 0.5:
            j1 = int(self.position[1]) + 1
        else:
            j1 = int(self.position[1])
        i2 = i1 + 1
        j2 = j1 + 1
        intensity1 = pheromone.get_intensity(i1, j1)
        intensity2 = pheromone.get_intensity(i2, j1)
        intensity3 = pheromone.get_intensity(i1, j2)
        intensity4 = pheromone.get_intensity(i2, j2)
        total_intensity = intensity1 + intensity2 + intensity3 + intensity4
        if total_intensity == 0:
            return [0, 0]
        else:
            x = (i1*intensity1 + i2*intensity2 + i1*intensity3 + i2*intensity4) / total_intensity
            y = (j1*intensity1 + j1*intensity3 + j2*intensity2 + j2*intensity4) / total_intensity
            dx = self.position[0] - x
            dy = self.position[1] - y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance == 0:
                return [random.uniform(-1, 1), random.uniform(-1, 1)]
            else:
                return [-dx/distance, -dy/distance]

    def apply_force(self, force):
        if force[0] != 0 or force[1] != 0:
            force_mag = math.sqrt(force[0]*force[0] + force[1]*force[1])
            if force_mag > self.max_force:
                force[0] *= self.max_force / force_mag
                force[1] *= self.max_force / force_mag
            self.velocity[0] += force[0]
            self.velocity[1] += force[1]
            
    def limit_speed(self):
        speed = math.sqrt(self.velocity[0]*self.velocity[0] + self.velocity[1]*self.velocity[1])
        if speed > self.max_speed:
            self.velocity[0] *= self.max_speed / speed
            self.velocity[1] *= self.max_speed / speed



pygame.init()

screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

grid_width = 100
grid_height = 100
cell_width = screen_width // grid_width
cell_height = screen_height // grid_height

pheromones = [Pheromone(grid_width, grid_height)]

particles = []
for i in range(50):
    x = random.uniform(0, screen_width)
    y = random.uniform(0, screen_height)
    particle = Particle(x, y)
    particles.append(particle)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
            
            
    # Update pheromones
    for pheromone in pheromones:
        pheromone.update()
        pheromone.deposit(random.uniform(0, screen_width), random.uniform(0, screen_height), 10.0)

    # Update particles
    for particle in particles:
        particle.update(pheromones)

    # Draw pheromones
    for pheromone in pheromones:
        for i in range(grid_width):
            for j in range(grid_height):
                intensity = pheromone.get_intensity(i, j)
                if intensity > 0:
                    color = (255, 255 - int(intensity), 255 - int(intensity))
                    pygame.draw.rect(screen, color, (i*cell_width, j*cell_height, cell_width, cell_height))

    # Draw particles
    for particle in particles:
        pygame.draw.circle(screen, (255, 0, 0), (int(particle.position[0]), int(particle.position[1])), int(particle.radius))

    pygame.display.flip()
    clock.tick(60)
pygame.quit()