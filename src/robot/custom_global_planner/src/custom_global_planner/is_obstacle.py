#!/usr/bin/env python3

#function for opredeleniya is this point obstacle or not\
#true -- point is obstacle
#false -- point free
import numpy as np
import where_square, obstacle_coordinate
def is_obstacle(costmap, costmap_resolution, x, y, ror):
    cost_mat_cells = np.array(costmap)
    height, width = cost_mat_cells.shape
    [x_o, y_o] = obstacle_coordinate.obstacle_coordinate(costmap)
    flag = False
#    if cost_mat_cells[int(y)][int(x)] > 75:
    for i in range(len(x_o)):
        if (where_square.where_square(x_o[i] - ror, y_o[i] + ror, costmap_resolution + 2*ror, x, y) == np.array([0,0])).all():
            flag = True
            break
    return flag
