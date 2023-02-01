import matplotlib
# matplotlib.use('QT5Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import random
import keyboard
import math

# --- classes ---


class Particle:
    def __init__(self):
        self.x = 5 * np.random.random_sample()
        self.y = 5 * np.random.random_sample()
        #self.vx = 5 * np.random.random_sample() - 0.5 / 5
        #self.vy = 5 * np.random.random_sample() - 0.5 / 5
        # self.vx = np.random.random_sample() / 5
        # self.vy = np.random.random_sample() / 5
        self.vx = 0.1
        self.vy = 0.1
        self.angle = np.angle(self.vy/self.vx)


    def move(self):
        vx = self.vx * math.cos(self.angle)
        vy = self.vy * math.sin(self.angle)
        
        if self.x < 0: self.x += self.vx
        if self.x >= 5: self.x -= self.vx
        if self.y < 0: self.y += self.vy
        if self.y >= 5: self.y -= self.vy
        
        #move forward
        if keyboard.is_pressed('up'):
            self.y += vy
            self.x += vx
        if keyboard.is_pressed('down'):
            self.y -= vy
            self.x -= vx 
        if keyboard.is_pressed('right'):
            self.angle -= 0.1
        if keyboard.is_pressed('left'):
            self.angle += 0.1

        
        # if keyboard.is_pressed('up'):
        #     self.y += self.vy
        # if keyboard.is_pressed('down'):
        #     self.y -= self.vy
        # if keyboard.is_pressed('right'):
        #     self.x += self.vx
        # if keyboard.is_pressed('left'):
        #     self.x -= self.vx
        


# --- functions ---

def animate(frame_number):
    global d  # need it to remove old plot

    # print('frame_number:', frame_number)
    # print()

    # move all particles
    for pi in pop:
        pi.move()

    # after for-loop

    # remove old plot
    #d.set_data([], [])
    d.remove()

    # create new plot
    d, = plt.plot([particle.x for particle in pop], [
                  particle.y for particle in pop], 'go')

# --- main ---
population = 2

pop = [Particle() for i in range(population)]

fig = plt.gcf()
# draw first plot
d,  = plt.plot([particle.x for particle in pop], [
               particle.y for particle in pop], 'go')
anim = animation.FuncAnimation(
    fig, animate, frames=200, interval=45, repeat=False)

plt.show()


# anim.save('particles.gif', fps=25)
#anim.save('particles.gif', writer='ffmpeg', fps=25)
#anim.save('particles.gif', writer='imagemagick', fps=25)
