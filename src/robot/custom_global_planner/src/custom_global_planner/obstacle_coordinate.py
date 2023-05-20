#!/usr/bin/env python3
#find coordinates of obstacles in metres
import numpy as np

def obstacle_coordinate(costmap):
    cost_mat_cells = np.array(costmap)
    height, width = cost_mat_cells.shape
    
    x_o = []
    y_o = []
    
    for i in range (width):
        for j in range (height):
            if cost_mat_cells[j,i] > 250:
                x_o.append(i)
                y_o.append(height-j)
    return [x_o, y_o]
