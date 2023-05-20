#!/usr/bin/env python3
#переводит сообщение из топика costmap в двумерную матрицу
import numpy as np

def msg2mat(array, height, width):
    costmap = np.zeros(15, 10)
    print(costmap)
    j = 0 #number column
    k = 0 #number string
    for i in range(len(array)):
        costmap[k][j] = array[i]
        j+=1
        if j == width - 1: #becouse we count from 0
            j = 0
            k+=1
    return costmapik
        
