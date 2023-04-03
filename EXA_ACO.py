import pygame
import random
import math

# Define constants
WIDTH = 800
HEIGHT = 600
FPS = 60
ANT_SIZE = 5
ANT_SPEED = 2
PHEROMONE_DECAY = 0.1
PHEROMONE_DEPOSIT = 10
MAX_PHEROMONE = 100

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
GREEN = (0, 255, 0)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Define the Ant class
class Ant(pygame.sprite.Sprite):
    def __init__(self, x, y):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface((ANT_SIZE, ANT_SIZE))
        self.image.fill(GRAY)
        self.rect = self.image.get_rect()
        self.rect.x = x
        self.rect.y = y
        self.direction = random.uniform(0, 2 * math.pi)
        self.pheromone_level = 0

    def update(self, pheromone_grid):
        # Move the ant
        dx = ANT_SPEED * math.cos(self.direction)
        dy = ANT_SPEED * math.sin(self.direction)
        self.rect.x += dx
        self.rect.y += dy

        # Wrap the ant around the screen if it goes off the edge
        if self.rect.x < 0:
            self.rect.x = WIDTH + self.rect.x
        elif self.rect.x > WIDTH:
            self.rect.x = self.rect.x - WIDTH
        if self.rect.y < 0:
            self.rect.y = HEIGHT + self.rect.y
        elif self.rect.y > HEIGHT:
            self.rect.y = self.rect.y - HEIGHT

        # Deposit pheromone
        x = self.rect.x // ANT_SIZE
        y = self.rect.y // ANT_SIZE
        self.pheromone_level += PHEROMONE_DEPOSIT
        pheromone_grid[x][y] = min(MAX_PHEROMONE, pheromone_grid[x][y] + self.pheromone_level)

        # Follow pheromone trail
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                nx = (x + i) % (WIDTH // ANT_SIZE)
                ny = (y + j) % (HEIGHT // ANT_SIZE)
                neighbors.append((nx, ny, pheromone_grid[nx][ny]))
        neighbors.sort(key=lambda x: -x[2])
        if neighbors:
            nx, ny, _ = neighbors[0]
            dx = nx * ANT_SIZE - self.rect.x
            dy = ny * ANT_SIZE - self.rect.y
            self.direction = math.atan2(dy, dx)

        # Decay pheromone trail
        self.pheromone_level -= PHEROMONE_DECAY
        if self.pheromone_level < 0:
            self.pheromone_level = 0
        pheromone_grid[x][y] = min(MAX_PHEROMONE, pheromone_grid[x][y] +self.pheromone_level)
         # Draw the ant
    def draw(self, screen):
        pygame.draw.circle(screen, WHITE, (self.rect.x, self.rect.y), ANT_SIZE)
def main():
    # Create the pheromone grid
    pheromone_grid = [[0 for _ in range(HEIGHT // ANT_SIZE)] for _ in range(WIDTH // ANT_SIZE)]
    # Create the ants
    ants = []
    for i in range(10):
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        ant = Ant(x, y)
        ants.append(ant)

    # Start the main loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the ants
        for ant in ants:
            ant.update(pheromone_grid)

        # Draw the ants and pheromone trail
        screen.fill(BLACK)
        for ant in ants:
            ant.draw(screen)
        for x in range(WIDTH // ANT_SIZE):
            for y in range(HEIGHT // ANT_SIZE):
                level = pheromone_grid[x][y]
                if level > 0:
                    alpha = min(255, level * 2)
                    color = GREEN + (alpha,)
                    rect = pygame.Rect(x * ANT_SIZE, y * ANT_SIZE, ANT_SIZE, ANT_SIZE)
                    pygame.draw.rect(screen, color, rect)

        # Update the display
        pygame.display.flip()
        

        # Wait for the next frame
        clock.tick(FPS)

main()