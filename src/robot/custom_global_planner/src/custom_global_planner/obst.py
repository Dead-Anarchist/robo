#!/usr/bin/env python3
import numpy as np
import where_square, min_skal
#find obstacles in half of cicle on our way, return list of perpendikulars from us to obstacles on our eyes
def dofind(enemy, x_r, y_r, dg, ror, costmap_resolution):    
    short_enemy = []
    for i in range (0, len(enemy)):
        #enemy - obstacle on our eyes. count the most little norms to enemy
        short_enemy.append(where_square.where_square(enemy[i][0]-ror, enemy[i][1]+ror, costmap_resolution+2*ror, x_r, y_r))

    return short_enemy
