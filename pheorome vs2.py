
import numpy as np
import random

# the grid system
grid = np.zeros((100, 100), dtype=int)

# obstacles just set the num between 100 to 1000
num_obstacles = random.randint(100, 1000)
obstacle_indices = random.sample(range(10000), num_obstacles)
obstacle_coords = [(i // 100, i % 100) for i in obstacle_indices]
for coord in obstacle_coords:
    grid[coord] = -1

# the pheromone map
pheromones = np.zeros((100, 100), dtype=int)

# the visited cells map
visited = np.zeros((100, 100), dtype=bool)

# a random starting position, if need to set starting point (0,0) then use pos = (0, 0)
pos = (random.randint(0, 99), random.randint(0, 99))

# marking the initial position with pheromones
pheromones[pos] = 100

# function that updating the pheromone map
def update_pheromones(pheromones, visited):
    for i in range(100):
        for j in range(100):
            if visited[i, j]:
                pheromones[i, j] = 100
            else:
                pheromones[i, j] = max(0, pheromones[i, j] - 1)

# getting neighboring cells
def get_neighbors(pos):
    i, j = pos
    neighbors = []
    if i > 0 and grid[i-1, j] != -1:
        neighbors.append((i-1, j))
    if i < 99 and grid[i+1, j] != -1:
        neighbors.append((i+1, j))
    if j > 0 and grid[i, j-1] != -1:
        neighbors.append((i, j-1))
    if j < 99 and grid[i, j+1] != -1:
        neighbors.append((i, j+1))
    return neighbors

# function to get the neighboring cells with the lowest pheromone value
def get_min_neighbors(pos, pheromones):
    neighbors = get_neighbors(pos)
    min_val = np.inf
    min_neighbors = []
    for n in neighbors:
        if pheromones[n] < min_val:
            min_val = pheromones[n]
            min_neighbors = [n]
        elif pheromones[n] == min_val:
            min_neighbors.append(n)
    return min_neighbors

# Main loop
while not np.all(visited):
    visited[pos] = True
    update_pheromones(pheromones, visited)
    min_neighbors = get_min_neighbors(pos, pheromones)
    if not min_neighbors:
        break
    next_pos = random.choice(min_neighbors)
    pos = next_pos

# Print the visited cells map
print(visited.astype(int))