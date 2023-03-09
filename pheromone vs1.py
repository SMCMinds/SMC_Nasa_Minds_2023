import random

ROWS = 100
COLS = 100

class Cell:
    def __init__(self):
        self.obstacle = 0
        self.pheromone = 0
        self.visited = 0

grid = [[Cell() for _ in range(COLS)] for _ in range(ROWS)]

# randomly generate obstacles
def generateObstacles(numObstacles):
    for i in range(numObstacles):
        row = random.randint(0, ROWS-1)
        col = random.randint(0, COLS-1)
        grid[row][col].obstacle = 1

# initialize the pheromone map to 0
def initPheromones():
    for i in range(ROWS):
        for j in range(COLS):
            grid[i][j].pheromone = 0

# updating the pheromone map based on the robot's path
def updatePheromones(pathX, pathY):
    for i in range(len(pathX)):
        row = pathX[i]
        col = pathY[i]
        grid[row][col].pheromone = 100

# updating the pheromone map over time
def evaporatePheromones(decayRate):
    for i in range(ROWS):
        for j in range(COLS):
            if grid[i][j].pheromone > 0:
                grid[i][j].pheromone -= decayRate

# updating the visited map based on the robot's path
def updateVisited(pathX, pathY):
    for i in range(len(pathX)):
        row = pathX[i]
        col = pathY[i]
        grid[row][col].visited = 1

# print the grid
def printGrid():
    for i in range(ROWS):
        for j in range(COLS):
            if grid[i][j].obstacle == 1:
                print("*", end="")
            elif grid[i][j].visited == 1:
                print(".", end="")
            else:
                print(" ", end="")
        print()

# checking a cell whether it is a valid neighbor for the robot to move to
def isValidNeighbor(row, col):
    if row < 0 or row >= ROWS or col < 0 or col >= COLS:
        return False    # out of bounds
    if grid[row][col].obstacle == 1 or grid[row][col].visited == 1:
        return False    # obstacle or already visited
    return True

# get the minimum pheromone value among the neighbors of a cell
def getMinNeighborPheromone(row, col):
    neighbors = []
    if isValidNeighbor(row-1, col):
        neighbors.append((row-1, col))
    if isValidNeighbor(row+1, col):
        neighbors.append((row+1, col))
    if isValidNeighbor(row, col-1):
        neighbors.append((row, col-1))
    if isValidNeighbor(row, col+1):
        neighbors.append((row, col+1))
    minPheromone = min([grid[r][c].pheromone for r, c in neighbors])
    minNeighbors = [(r, c) for r, c in neighbors if grid[r][c].pheromone == minPheromone]
    return random.choice(minNeighbors) if minNeighbors else None

# generate a random starting position for the robot
def generateStartingPosition():
    while True:
        row = random.randint(0, ROWS-1)
        col = random.randint(0, COLS-1)
        if not grid[row][col].obstacle:
            return row, col

# Main function to run the algorithm
def runAlgorithm(decayRate):
    # Randomly generate a number of obstacles
    numObstacles = random.randint(1, ROWS*COLS)
    generateObstacles(numObstacles)
    
    # Generate a random starting position for the robot
    row, col = generateStartingPosition()
    
    # Initialize the visited map
    grid[row][col].visited = 1
    
    # Run the algorithm until all cells have been visited
    while True:
        # Move to the neighboring cell with the smallest pheromone value
        nextCell = getMinNeighborPheromone(row, col)
        if nextCell is None:
            break   # no valid neighbors left
        row, col = nextCell
        
        # Update the visited map
        grid[row][col].visited = 1
        
        # Update the pheromone map
        grid[row][col].pheromone = 100
        
        # Evaporate the pheromones over time
        evaporatePheromones(decayRate)
        
        # Check if all cells have been visited
        if all(grid[i][j].visited for i in range(ROWS) for j in range(COLS)):
            break