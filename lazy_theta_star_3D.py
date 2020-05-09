# 3D A star algorithm

import numpy as np
from line_sight_partial_3D import line_sight_partial_3D


def lazy_theta_star_3D(K, E3d_safe, x0_old, y0_old, z0_old, xend_old, yend_old, zend_old, sizeE):


    ##############################################################################################################
    # Cost weights

    kg = K[0]
    kh = K[1]
    ke = K[2]


    ##############################################################################################################
    # Algorithm

    # If start and end points are not integer, they are rounded for the path generation.
    # Use of floor and ceil so they are different even if very close
    x0 = np.int(np.floor(x0_old))
    y0 = np.int(np.floor(y0_old))
    z0 = np.int(np.floor(z0_old))
    xend = np.int(np.ceil(xend_old))
    yend = np.int(np.ceil(yend_old))
    zend = np.int(np.ceil(zend_old))


    # Initialize

    # Size of environment matrix
    y_size = sizeE[0]
    x_size = sizeE[1]
    z_size = sizeE[2]


    # Node from which the good neighbour is reached
    came_fromx = np.zeros((y_size, x_size, z_size))
    came_fromy = np.zeros((y_size, x_size, z_size))
    came_fromz = np.zeros((y_size, x_size, z_size))
    came_fromx[y0, x0, z0] = x0
    came_fromy[y0, x0, z0] = y0
    came_fromz[y0, x0, z0] = z0

    # Nodes already evaluated
    closed_list = np.array([])

    # Nodes to be evaluated
    open_list = np.array([[y0, x0, z0]])

    # Cost of moving from start point to the node
    G = float("inf") * np.ones((y_size, x_size, z_size))
    G[y0, x0, z0] = 0

    # Initial total cost
    F = float("inf") * np.ones((y_size, x_size, z_size))
    F[y0, x0, z0] = np.sqrt((xend-x0)**2+(yend-y0)**2+(zend-z0)**2)

    # Initialize
    f_open = np.array([[F[y0, x0, z0]]])
    exit_path = 0


    # While open is not empty and we have not reached last point
    while len(open_list) > 0 and exit_path == 0:

        # Find node with minimum f in open

        # Find the index location in open for the node with minimum f
        i_f_open_min = np.argmin(f_open)

        # Location of node with minimum f in open list
        ycurrent = open_list[i_f_open_min, 0]
        xcurrent = open_list[i_f_open_min, 1]
        zcurrent = open_list[i_f_open_min, 2]

        # Define line of sight evaluation points
        xb = [came_fromx[ycurrent, xcurrent, zcurrent], xcurrent]
        yb = [came_fromy[ycurrent, xcurrent, zcurrent], ycurrent]
        zb = [came_fromz[ycurrent, xcurrent, zcurrent], zcurrent]

        # Line of sight check between coming point(parent) and neighbour
        sight = line_sight_partial_3D(E3d_safe, xb, yb, zb, sizeE)


        # If there is no line of sight
        if sight == 0 or sight == 0.5:

            # Initialize minimum g value
            g_min = float("inf")

            # Check neighbour nodes
            for i in range(-1, 2):
                for j in range(-1, 2):
                    for k in range(-1, 2):


                        # If the neighbour node is within the grid
                        if 0 <= xcurrent + i < x_size and 0 <= ycurrent + j < y_size and 0 <= zcurrent + k < z_size:

                            # If the neighbour belongs to closed lists
                            neigh = np.array([ycurrent + j, xcurrent + i, zcurrent + k])

                            sum_closed = np.sum(neigh == closed_list, 1)
                            if len(sum_closed) > 0:
                                max_sum_closed = max(sum_closed)
                            else:
                                max_sum_closed = 0

                            if max_sum_closed == 3:

                                # Add the neighbour node to open list
                                open_list = np.vstack((open_list, neigh))


                                # Evaluate the distance from start to neighbour node + from the neighbour node to the current node
                                g_try = G[ycurrent + j, xcurrent + i, zcurrent + k] + np.sqrt(i ** 2 + j ** 2 + k ** 2)

                                # If this distance is smallest, save it and assign neighbour as parent node
                                if g_try < g_min:

                                    g_min = g_try
                                    G[ycurrent, xcurrent, zcurrent] = g_try

                                    # Record from which node the good neighbour is reached
                                    came_fromy[ycurrent, xcurrent, zcurrent] = ycurrent + j
                                    came_fromx[ycurrent, xcurrent, zcurrent] = xcurrent + i
                                    came_fromz[ycurrent, xcurrent, zcurrent] = zcurrent + k

                                f_open = np.vstack((f_open, [F[ycurrent + j, xcurrent + i, zcurrent + k]]))



        # Check if the arrival node is reached
        if xcurrent == xend and ycurrent == yend and zcurrent == zend:

            # Arrival node reached, exit and generate path
            exit_path = 1

        else:

            # Add the node to closed list
            if closed_list.shape[0] == 0:
                closed_list = np.array([[ycurrent, xcurrent, zcurrent]])
            else:
                closed_list = np.vstack((closed_list, np.array([ycurrent, xcurrent, zcurrent])))

            # Remove the node from open list
            open_list = np.delete(open_list, i_f_open_min, 0)
            f_open = np.delete(f_open, i_f_open_min, 0)


            # Check neighbour nodes
            for i in range(-1, 2):
                for j in range(-1, 2):
                    for k in range(-1, 2):

                        # If the neighbour node is within the grid
                        if 0 <= xcurrent + i < x_size and 0 <= ycurrent + j < y_size and 0 <= zcurrent + k < z_size:

                            # If the neighbour node does not belong to open and to closed lists
                            neigh = np.array([ycurrent + j, xcurrent + i, zcurrent + k])

                            sum_open = np.sum(neigh == open_list, 1)
                            sum_closed = np.sum(neigh == closed_list, 1)

                            if len(sum_open) > 0:
                                max_sum_open = max(sum_open)
                            else:
                                max_sum_open = 0

                            if len(sum_closed) > 0:
                                max_sum_closed = max(sum_closed)
                            else:
                                max_sum_closed = 0

                            if max_sum_open < 3 and max_sum_closed < 3:

                                # Add the neighbour node to open list
                                open_list = np.vstack((open_list, neigh))

                                # Evaluate the distance from start to coming point(parent)+ from the coming point(parent) to the neighbour of the current node
                                yg = np.int(came_fromy[ycurrent, xcurrent, zcurrent])
                                xg = np.int(came_fromx[ycurrent, xcurrent, zcurrent])
                                zg = np.int(came_fromz[ycurrent, xcurrent, zcurrent])
                                g_try = G[yg, xg, zg] + np.sqrt((came_fromy[ycurrent, xcurrent, zcurrent] - (ycurrent + j)) ** 2 + (came_fromx[ycurrent, xcurrent, zcurrent] - (xcurrent + i)) ** 2 + (came_fromz[ycurrent, xcurrent, zcurrent] - (zcurrent + k)) ** 2)

                                # If this distance is smaller than the neighbour distance
                                if g_try < G[ycurrent + j, xcurrent + i, zcurrent + k]:
                                    # We are on the good path, save information

                                    # Record from which node the good neighbour is reached
                                    came_fromy[ycurrent + j, xcurrent + i, zcurrent + k] = came_fromy[ycurrent, xcurrent, zcurrent]
                                    came_fromx[ycurrent + j, xcurrent + i, zcurrent + k] = came_fromx[ycurrent, xcurrent, zcurrent]
                                    came_fromz[ycurrent + j, xcurrent + i, zcurrent + k] = came_fromz[ycurrent, xcurrent, zcurrent]

                                    # Evaluate the cost function
                                    G[ycurrent + j, xcurrent + i, zcurrent + k] = g_try
                                    H = np.sqrt((xend - (xcurrent + i)) ** 2 + (yend - (ycurrent + j)) ** 2 + (zend - (zcurrent + k)) ** 2)
                                    F[ycurrent + j, xcurrent + i, zcurrent + k] = kg * G[ycurrent + j, xcurrent + i, zcurrent + k] + kh * H + ke * E3d_safe[ycurrent + j, xcurrent + i, zcurrent + k]


                                f_open = np.vstack((f_open, [F[ycurrent + j, xcurrent + i, zcurrent + k]]))



    # Reconstruct path backwards knowing from which node the good neighbour is reached

    # First element is the arrival point
    path_backwards = [ycurrent, xcurrent, zcurrent]

    # Initialize
    i = 1

    # While the start point is not reached
    while (xcurrent != x0) or (ycurrent != y0) or (zcurrent != z0):

        next_point = np.array([came_fromy[ycurrent, xcurrent, zcurrent], came_fromx[ycurrent, xcurrent, zcurrent], came_fromz[ycurrent, xcurrent, zcurrent]])
        path_backwards = np.vstack((path_backwards, next_point))

        ycurrent = np.int(path_backwards[i, 0])
        xcurrent = np.int(path_backwards[i, 1])
        zcurrent = np.int(path_backwards[i, 2])

        i = i + 1


    # Number of waypoints
    n_points = path_backwards.shape[0]

    # Reverse path sequence
    path = np.flipud(path_backwards)

    # Reassign the original start and end points
    path[0, :] = [y0_old, x0_old, z0_old]
    path[-1, :] = [yend_old, xend_old, zend_old]


    return [path, n_points]
