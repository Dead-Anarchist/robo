#!/usr/bin/env python3

import numpy as np
#input is lists:
# xs - list of x
# ys - list of y
# ori_start ori_end - orientations 

#new orientaton is orientation of sum of vectors (previous - preprevious) and (now - previous)
#output id list of orientations incuding start and end from input. angle of orientation in radians
def orient(xs, ys, ori_start, ori_end):
    ori = [ori_start] + [0.] * (len(xs)-2) + [ori_end]
    for i in range(1, len(xs)-1): #begin from 1, because 0 point has ori const. the last point (goal) too
        dx = xs[i+1] - xs[i-1]
        dy = ys[i+1] - ys[i-1]
        ori[i] = np.arctan2(dy, dx)
    return ori
