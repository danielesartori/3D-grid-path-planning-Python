# Main file

import numpy as np
import time
import matplotlib.pyplot as plt
from grid_3D_safe_zone import grid_3D_safe_zone
from a_star_3D import a_star_3D
from theta_star_3D import theta_star_3D


##############################################################################################################
# Grid and path parameters

# Grid size and resolution
sizeE = [80, 80, 20]
d_grid = 1

# Starting point
x0 = 5
y0 = 7
z0 = 10

# Arrival point
xend = 68
yend = 66
zend = 6

# Number of points with low elevation around start and end point area
n_low = 3

# A* or Theta* cost weights
kg = 1
kh = 1.25
ke = np.sqrt((xend-x0)**2+(yend-y0)**2+(zend-z0)**2)


##############################################################################################################
# Map definition

# Average flight altitude
h = max(z0, zend)

# Points coordinates in [y,x,z] format
P0 = np.array([y0, x0, z0])
Pend = np.array([yend, xend, zend])

# Generate map
[E, E_safe, E3d, E3d_safe] = grid_3D_safe_zone(sizeE, d_grid, h, P0, Pend, n_low)


##############################################################################################################
# Path generation

# Store gains in vector
K = [kg, kh, ke]

# Start measuring execution time
start_time = time.time()

# Generate path (chose one)
# [path, n_points] = a_star_3D(K, E3d_safe, x0, y0, z0, xend, yend, zend, sizeE)
[path, n_points] = theta_star_3D(K, E3d_safe, x0, y0, z0, xend, yend, zend, sizeE)

# Stop measuring and print execution time
print(" %s seconds" % (time.time() - start_time))


##############################################################################################################
# Figure

X = np.arange(1, sizeE[0]-1, d_grid)
Y = np.arange(1, sizeE[1]-1, d_grid)
X, Y = np.meshgrid(X, Y)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, E[1:-1][:, 1:-1])
ax.plot(path[:][:, 1], path[:][:, 0], path[:][:, 2], 'kx-')
ax.plot([x0], [y0], [z0], 'go')
ax.plot([xend], [yend], [zend], 'ro')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.auto_scale_xyz([0, sizeE[1]], [0, sizeE[0]], [0, sizeE[2]])
plt.show()
