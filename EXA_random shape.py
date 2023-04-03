import pygame
import random
import math
# Initialize Pygame
pygame.init()

# Define the dimensions of the screen
WIDTH = 800
HEIGHT = 600

# Set up the Pygame display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Random Rock Shape")

# Define the colors
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
WHITE = (255, 255, 255)

# Define a function to generate a random rock shape
def generate_rock_shape(size):
    # Generate a list of random angles and sort them in ascending order
    angles = [random.uniform(0, 2 * math.pi) for i in range(10)]
    angles.sort()

    # Generate a list of random radii
    radii = [random.uniform(size / 2, size) for i in range(10)]

    # Calculate the x and y coordinates for each point
    points = []
    for i in range(10):
        angle = angles[i]
        radius = radii[i]
        x = size + radius * math.cos(angle)
        y = size + radius * math.sin(angle)
        points.append((x, y))

    # Check if the polygon is convex, and if not, generate a new polygon
    if not is_polygon_convex(points):
        return generate_rock_shape(size)
    else:
        return points
        
# Define a function to check if a polygon is convex
def is_polygon_convex(points):
    # Check if the polygon has at least 3 points
    if len(points) < 3:
        return False

    # Calculate the cross product of each adjacent pair of edges
    cross_products = []
    for i in range(len(points)):
        v1 = points[(i + 1) % len(points)][0] - points[i][0], points[(i + 1) % len(points)][1] - points[i][1]
        v2 = points[(i + 2) % len(points)][0] - points[(i + 1) % len(points)][0], points[(i + 2) % len(points)][1] - points[(i + 1) % len(points)][1]
        cross_products.append(v1[0] * v2[1] - v1[1] * v2[0])

    # If all cross products have the same sign, the polygon is convex
    if all(cross_product > 0 for cross_product in cross_products) or all(cross_product < 0 for cross_product in cross_products):
        return True
    else:
        return False

# Generate Rocks
rock_points = generate_rock_shape(random.randint(50, 150))


# Define the main game loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Fill the background color
    screen.fill(BLACK)
    
    # Draw the rock shape
    
    pygame.draw.polygon(screen, GRAY, rock_points)
    for point in rock_points:
        pygame.draw.circle(screen, WHITE, (int(point[0]), int(point[1])), 2)
    
    # Update the screen
    pygame.display.update()


