#!/usr/bin/env python3
#find vector from point to qwadrat. 

import numpy as np

def where_square(x_sq, y_sq, a, x_r, y_r): #x_sq, y_sq - left up point of squere, a - side of squere, x_r, y_r - coordinates of robot. Result is vector with begin in (x_r, y_r) and end in nearest point on squere perimetr. if robot in squere, output in null vector
    
    point_sq = [ [x_sq, y_sq], [x_sq + a, y_sq], [x_sq + a, y_sq - a], [x_sq, y_sq - a] ] #left up, right up, right down, left down points accordingly
    
    '''                   1 |    2    |3
                        ____|_________|______
                            |         |
                          4 |    5    |6
                            |         |            
                        ____|_________|_____
                          7 |    8    |9
                            |         |
    '''
    res = [0, 0] #result
    if (x_r <= x_sq + a) and (x_r >= x_sq) and (y_r <= y_sq) and (y_r >= y_sq - a): #zone 5
        res = [0, 0]
    
    if (x_r <= x_sq + a) and (x_r >= x_sq):
        if y_r > y_sq: #zone 2
            res = [0, y_sq  - y_r]
        if y_r < y_sq - a: #zone 8
            res = [0, y_sq - a - y_r]
        
    if (y_r <= y_sq) and (y_r >= y_sq - a):
        if x_r > x_sq + a: #zone 6
            res = [x_sq + a - x_r, 0]
        if x_r < x_sq: #zone 4
            res = [x_sq - x_r, 0]
    
    if y_r > y_sq:
        if x_r < x_sq: #zone 1
            res = [x_sq - x_r, y_sq - y_r]
        if x_r > x_sq + a: #zone 3
            res = [x_sq + a - x_r, y_sq - y_r]
    
    if y_r < y_sq - a:
        if x_r < x_sq: #zone 7
            res = [x_sq - x_r, y_sq - a - y_r]
        if x_r > x_sq + a: #zone 9
            res = [x_sq + a - x_r, y_sq - a - y_r]
    
    return np.array(res)
