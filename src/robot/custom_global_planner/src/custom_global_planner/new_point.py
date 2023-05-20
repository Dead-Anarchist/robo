#!/usr/bin/env python3

import du
from scipy.integrate import odeint
from scipy import integrate
import numpy as np
#sole DU and count new x_r, y_r

def new_point(velocity_x, velocity_y, x_r, y_r, t, u, T, k):
    r0 = np.array([velocity_x[-1], velocity_y[-1]])
    r_sol = odeint(lambda x, time_: du.func(x, time_, np.array(u), T, k), r0, t)
    velocity_x = r_sol[:,0]
    x_r += integrate.trapz(velocity_x, t)
    velocity_y = r_sol[:,1]
    y_r += integrate.trapz(velocity_y, t)
    return (x_r, y_r)   
