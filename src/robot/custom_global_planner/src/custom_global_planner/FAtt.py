#!/usr/bin/env python3
import numpy as np


#count Fatt - power of attractive
#dg - vector-string from robot to goal
#ka - for attractive power
#rog - radius of zone near goal, where we can use parabolic potention field

def FAtt(dg, x_goal, y_goal, x_r, y_r, ka, rog):
    if np.linalg.norm(dg) <= rog:
        Fatt = ka * dg
    else:
        Fatt = ka * dg / np.linalg.norm(dg)

    return Fatt
