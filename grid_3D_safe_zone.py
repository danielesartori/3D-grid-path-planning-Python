# 3D grid map generator

import numpy as np
from numpy.random import default_rng


def grid_3D_safe_zone(sizeE, d_grid, h, P0, Pend, n_low):


    # Grid  size
    y_size = sizeE[0]
    x_size = sizeE[1]
    z_size = sizeE[2]

    # Vertical grid vector
    z_grid = np.linspace(1, z_size, z_size//d_grid)


    ##############################################################################################################
    # Random grid generation, discrete height in big blocks

    mean_E = 0
    sigma = 1
    k_sigma = 2.25   # edit this to change obstacles density
    rng = default_rng()
    E = rng.normal(mean_E, sigma, (y_size, x_size))
    sigma_obstacle = k_sigma * sigma
    E = np.uint8(E > sigma_obstacle)


    # Assign random altitude to blocks around points

    # Minimum obstacle altitude
    hh_min = 3

    # Initialize temporary matrix for evaluation
    EE = E.copy()

    for i in range(x_size):
        for j in range(y_size):

            # Block boundaries with random dimension (max -2:+2)
            k = np.arange(i - 1 - round(rng.beta(0.5, 0.5)), i + 1 + round(rng.beta(0.5, 0.5)) + 1)
            l = np.arange(j - 1 - round(rng.beta(0.5, 0.5)), j + 1 + round(rng.beta(0.5, 0.5)) + 1)

            # If block boundaries within the grid and if the node point value is high,
            if min(k) >= 0 and min(l) >= 0 and max(k) < x_size and max(l) < y_size and EE[j, i] == 1:

                # Assign random value to block
                hh = round(rng.normal(0.75*h, 0.5*h))

                # Give a minimum value to the altitude and limit maximum altitude
                if hh < hh_min:
                    hh = hh_min
                elif hh > z_size:
                    hh = z_size

                E[np.ix_(l, k)] = hh * np.ones((len(l), len(k)))


    # Assign low elevation to and around start and end points
    E[np.ix_(np.arange(P0[0] - n_low, P0[0] + n_low + 1), np.arange(P0[1] - n_low, P0[1] + n_low + 1))] = 0
    E[np.ix_(np.arange(Pend[0] - n_low, Pend[0] + n_low + 1), np.arange(Pend[1] - n_low, Pend[1] + n_low + 1))] = 0


    ##############################################################################################################
    # Create 3D grid

    # Initialize
    E3d = np.zeros((y_size, x_size, z_size))

    # Create grid occupancy index (0=free, 1=occupied)
    for i in range(z_size):
        E3d[np.ix_(np.arange(0, y_size), np.arange(0, x_size), [i])] = np.atleast_3d(E[0:y_size][:, 0:x_size] >= z_grid[i])


    ##############################################################################################################
    # Create safe zone near high elevation elements

    # Initialize
    E_safe = E.copy()

    for i in range(x_size):
        for j in range(y_size):

            # Check neighbour nodes
            k = np.arange(i - 1, i + 2)
            l = np.arange(j - 1, j + 2)

            # Limit neighbours within the grid
            if min(k) < 0:
                k = np.arange(i, i + 2)
            elif max(k) >= x_size:
                k = np.arange(i - 1, i + 1)
            if min(l) < 0:
                l = np.arange(j, j + 2)
            elif max(l) >= y_size:
                l = np.arange(j - 1, j + 1)

            # Evaluation matrix
            E_eval = E[l][:, k]

            # If we are in a free point and nearby there is an obstacle
            if E[j, i] == 0 and E_eval.max() > 0:
                # Assign the maximum value of the neighbour nodes
                E_safe[j, i] = E_eval.max()


            # If point is elevated, add one safe step in altitude
            if 0 < E_safe[j, i] < (z_size - 1):
                E_safe[j, i] = E_safe[j, i] + 1


    ##############################################################################################################
    # Create grid occupancy index (0=free, 0.5=safe, 1=occupied)

    # Initialize
    E3d_safe = E3d.copy()

    for i in range(x_size):
        for j in range(y_size):
            for k in range(z_size):

                # Check neighbour nodes
                l = np.arange(i - 1, i + 2)
                m = np.arange(j - 1, j + 2)
                n = np.arange(k - 1, k + 2)

                # Limit neighbours within the grid
                if min(l) < 0:
                    l = np.arange(i, i + 2)
                elif max(l) >= x_size:
                    l = np.arange(i - 1, i + 1)
                if min(m) < 0:
                    m = np.arange(j, j + 2)
                elif max(m) >= y_size:
                    m = np.arange(j - 1, j + 1)
                if min(n) < 0:
                    n = np.arange(k, k + 2)
                elif max(n) >= z_size:
                    n = np.arange(k - 1, k + 1)

                # Evaluation matrix
                E_eval = E3d[m][:, l][:, :, n]

                # If we are in a free point and nearby there is an obstacle
                if E3d[j, i, k] == 0 and E_eval.max() == 1:
                    # Assign safe value
                    E3d_safe[j, i, k] = 0.5


    ##############################################################################################################
    # Create safe zone near borders

    E[np.ix_([0, -1], np.arange(0, y_size))] = z_size
    E[np.ix_(np.arange(0, x_size), [0, -1])] = z_size

    E_safe[np.ix_([0, -1], np.arange(0, y_size))] = z_size
    E_safe[np.ix_(np.arange(0, x_size), [0, -1])] = z_size

    E3d[np.ix_([0, -1], np.arange(0, y_size), np.arange(0, z_size))] = 1
    E3d[np.ix_(np.arange(0, x_size), [0, -1], np.arange(0, z_size))] = 1
    E3d[np.ix_(np.arange(0, x_size), np.arange(0, y_size), [0, -1])] = 1

    E3d_safe[np.ix_([0, -1], np.arange(0, y_size), np.arange(0, z_size))] = 1
    E3d_safe[np.ix_(np.arange(0, x_size), [0, -1], np.arange(0, z_size))] = 1
    E3d_safe[np.ix_(np.arange(0, x_size), np.arange(0, y_size), [0, -1])] = 1


    return [E, E_safe, E3d, E3d_safe]
