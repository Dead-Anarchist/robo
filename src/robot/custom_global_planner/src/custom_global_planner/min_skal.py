#!/usr/bin/env python3
#output is minimal vector to nearest obstacle
import where_square
import numpy as np
def min_skal(do_list, costmap_resolution):
        skals = []
        
        for i in range (0, len(do_list)):
            skals.append(where_square.where_square(do_list[i][0], do_list[i][1], costmap_resolution, 0,0))
        lengths = [np.linalg.norm(x) for x in skals]
        ind = lengths.index(min(lengths))
        return skals[ind]
