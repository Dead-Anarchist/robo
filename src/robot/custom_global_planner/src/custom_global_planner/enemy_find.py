#!/usr/bin/env python3
import numpy as np
def enemy_find(short_do_list, dg, ro0_pl, limcos):
    #find obstacles in half of cicle on our way
    short_enemy = []
    for i in range (0, len(short_do_list)):
        #obstacle on our eyes
        if ((np.dot(short_do_list[i], dg) >= 0) 
          and (np.linalg.norm(short_do_list[i]) <= ro0_pl) 
          and (np.linalg.norm(short_do_list[i]) <= np.linalg.norm(dg))
       #   and (np.dot(do_list[i], dg) / (np.linalg.norm(do_list[i]) * np.linalg.norm(dg)) <= limcos)
       ): #obslacte not behind goal
            short_enemy.append(short_do_list[i])
    return short_enemy
