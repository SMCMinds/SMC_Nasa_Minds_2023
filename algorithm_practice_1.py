#ploting using graphing method

import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-5.0, 5.0, 100)
y = np.linspace(-5.0, 5.0, 100)

# Creating 2-D grid of features
[X, Y] = np.meshgrid(x, y)

fig, ax = plt.subplots(1, 1)

Z = np.power(X**2+Y-11, 2)+np.power(X+Y**2-7, 2)

# plots contour lines
cp = ax.contour(X, Y, Z, 20)

# plots contour colors
# cp = ax.contourf(X, Y, Z, 20)
# fig.colorbar(cp)

# graph features
ax.set_title('Contour Plot')
ax.set_xlabel('feature_x')
ax.set_ylabel('feature_y')


a, b = np.gradient(Z)

# Size of grid is 100x100

#point1 at (4,4.9)
x = 4
y = 4.9
