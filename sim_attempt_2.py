import time
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.output = 0
    
    def update(self, process_variable):
        error = self.setpoint - process_variable
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return self.output

class LEDParticle:
    def __init__(self, x, y, speed, radius):
        self.x = x
        self.y = y
        self.speed = speed
        self.radius = radius
        self.history = [(x, y)]
    
    def move(self, dx, dy):
        self.x += dx
        self.y += dy
        self.history.append((self.x, self.y))
    
    def plot(self):
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'o-')
        ax.set_xlim(0, WIDTH)
        ax.set_ylim(0, HEIGHT)
        
        def update(frame):
            if not self.history:
                return line,
            x, y = zip(*self.history)
            line.set_data(x[:frame], y[:frame])
            return line,
        
        ani = animation.FuncAnimation(fig, update, frames=len(self.history)+1, interval=100, blit=True, repeat=False)
        plt.show()

    def detect_collision(self, obstacles):
        for obstacle in obstacles:
            if distance((self.x, self.y), (obstacle.x, obstacle.y)) < self.radius + obstacle.radius:
                return True
        return False


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

def distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5




def build_obstacle_grid(obstacles, grid_size, cell_size):
    grid = np.zeros((grid_size, grid_size))
    for obstacle in obstacles:
        x, y = int(obstacle.x / cell_size), int(obstacle.y / cell_size)
        r = int(obstacle.radius / cell_size)
        for i in range(-r, r+1):
            for j in range(-r, r+1):
                if 0 <= x+i < grid_size and 0 <= y+j < grid_size:
                    d = distance((x+i, y+j), (x, y))
                    if d <= r:
                        grid[y+j, x+i] += 1
    return grid


    def calculate_density_histogram(grid, sector_size):
        histogram = []
        for i in range(0, 360, sector_size):
            sector = grid[(i-sector_size//2)%360:(i+sector_size//2)%360, :]
            histogram.append(sector.sum)

    def generate_candidate_directions(density_histogram, threshold):
        candidate_directions = []
        for i in range(len(density_histogram)):
            if density_histogram[i] < threshold:
                candidate_directions.append(i)
        return candidate_directions

    def select_best_direction(candidate_directions, current_direction, sector_size):
        best_direction = current_direction
        min_difference = sector_size
        for direction in candidate_directions:
            difference = abs(direction - current_direction)
        if difference > 180:
            difference = 360 - difference
        if difference < min_difference:
            best_direction = direction
            min_difference = difference
        return best_direction

    def calculate_vector_field(candidate_directions, best_direction, sector_size):
        vector_field = np.zeros(360)
        for direction in candidate_directions:
            difference = abs(direction - best_direction)
        if difference > 180:
            difference = 360 - difference
            weight = 1 - (difference / (sector_size / 2))
            vector_field[direction] = weight
        return vector_field




WIDTH = 100
HEIGHT = 100
PARTICLE_RADIUS = 2
TARGET = (WIDTH/2, HEIGHT/2)
NUM_OBSTACLES = 20
OBSTACLE_RADIUS = 10
OBSTACLE_MIN_DIST = 20
SPEED = 1
KP = 0.5
KI = 0.0
KD = 0.1
SETPOINT = 0
SECTOR_SIZE = 30
THRESHOLD = 5


obstacles = []
for i in range(NUM_OBSTACLES):
    while True:
        x = random.uniform(OBSTACLE_RADIUS, WIDTH-OBSTACLE_RADIUS)
        y = random.uniform(OBSTACLE_RADIUS, HEIGHT-OBSTACLE_RADIUS)
        if all(distance((x, y), (o.x, o.y)) >= OBSTACLE_MIN_DIST + OBSTACLE_RADIUS for o in obstacles):
            obstacles.append(Obstacle(x, y, OBSTACLE_RADIUS))
            break


led_particle = LEDParticle(0, HEIGHT/2, SPEED, PARTICLE_RADIUS)
pid = PID(KP, KI, KD, SETPOINT)


while True:
    # Get the current position and calculate the distance to the target
    current_pos = (led_particle.x, led_particle.y)
    distance_to_target = distance(current_pos, TARGET)
    control_output = pid.update(distance_to_target)
   
        # Build the obstacle grid and density histogram
    obstacle_grid = build_obstacle_grid(obstacles, 100, 10)
    density_histogram = calculate_density_histogram(obstacle_grid, SECTOR_SIZE)

    # Generate candidate directions and select the best one
    candidate_directions = generate_candidate_directions(density_histogram, THRESHOLD)
    best_direction = select_best_direction(candidate_directions, int(pid.output), SECTOR_SIZE)

    # Calculate the vector field and select the direction with the highest weight
    vector_field = calculate_vector_field(candidate_directions, best_direction, SECTOR_SIZE)
    direction = np.argmax(vector_field)

    # Convert direction to movement
    dx = SPEED * np.cos(np.deg2rad(direction))
    dy = SPEED * np.sin(np.deg2rad(direction))

    # Move the LED particle
    led_particle.move(dx, dy)

    # Detect and avoid obstacles
    if led_particle.detect_collision(obstacles):
        print("Collision detected! Avoiding obstacle...")
        led_particle.move(-dx, -dy)

    # Plot the LED particle and obstacles
    led_particle.plot()
    for obstacle in obstacles:
        circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius)
                            
    # Update the PID controller with the error signal
    error = distance_to_target - SETPOINT
    pid.update(error)

    # Plot the target
    plt.scatter(TARGET[0], TARGET[1], marker='*', color='green')

    # Plot the vector field
    x = np.linspace(0, 2*np.pi, 360)
    y = vector_field
    plt.polar(x, y)

    # Display the plot
    plt.axis('equal')
    plt.xlim(0, WIDTH)
    plt.ylim(0, HEIGHT)
    plt.title("LED Particle Navigation with Collision Avoidance")
    plt.pause(0.001)
    plt.clf()

