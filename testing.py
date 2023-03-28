import pygame
import numpy as np


# a = pygame.Vector2(10,20)
# goal_pos = pygame.Vector2(50,50)
# print(a.reflect(goal_pos))
 
# a = a.move_towards(goal_pos, 80)
# print(a)

# a = 2
# b = a
# def func(a):
#     a = 1
#     return a
# func(b)
# print(a)

# a = [[1,1],[1,1]]
# print(sum(a))

import numpy as np
import matplotlib.pyplot as plt

x = np.zeros((1, 32))
y = np.zeros((1, 32))
arr = np.zeros((y.size, x.size))

cx = 10.
cy = 10.
r = 5.

# The two lines below could be merged, but I stored the mask
# for code clarity.
mask = (x[np.newaxis,:]-cx)**2 + (y[:,np.newaxis]-cy)**2 
mask1 = x[np.newaxis,:] + y[:,np.newaxis] 
print(mask)
arr[mask] += 123.
arr[mask] -= 50
# This plot shows that only within the circle the value is set to 123.
plt.figure(figsize=(6, 6))
plt.pcolormesh(x, y, arr)
plt.colorbar()
plt.show()