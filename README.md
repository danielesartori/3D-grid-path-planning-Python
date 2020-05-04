Some Python tools for path planning on a 3D grid map, including 3D A star

Main_3D.py: main file for defining the map and the path properties, to call the grid and path generation functions and to plot the results

Grid_3D_safe_zone.py: randomly generates a 3D cluttered environment (obstacles connected to the groud), which is represented both as a 2D matrix and as 3D occupancy map. For both maps, a safety buffer zone is created around and above obstacles

a_star_3D.py: 3D A* path planning algorithm
theta_star_3D.py: 3D Theta* path planning algorithm

New algorithms like 3D Lazy Theta* will be uploaded in the future
