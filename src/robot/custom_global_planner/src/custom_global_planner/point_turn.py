#!/usr/bin/env python3
import numpy as np
def point_turn(enemy, ro0_pl):
    r_pl = []
    #berem vector enemy, normiruem, otkladivaem ot prepyatstviya s - znakom
    if len(enemy)>0 and np.linalg.norm(enemy) != 0:
        for i in range (len(enemy[0])):
            enemy_norm = enemy/np.linalg.norm(enemy)
            r_pl = -ro0_pl * enemy_norm + enemy #point of bypass            
    return r_pl
