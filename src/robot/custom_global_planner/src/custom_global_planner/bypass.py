#!/usr/bin/env python3
import numpy as np
import min_skal, FByp, FAtt, point_turn, where_square
# получаем на вход координаты всех препятствий на карте, координаты робота, константы удаленности, константы для поля)
# выдаем результирующую силу для u 

def bypass(short_enemy, do, dg, costmap_resolution, x_r, y_r, x_goal, y_goal, ka, kr, rog, ror, ro0_pl, fmax, limcos, limturn):
    r_pl = point_turn.point_turn(short_enemy, ro0_pl)
    Fbyp = np.array([0., 0.])
    u = Fbyp
    if len(short_enemy) == 0:   
        u = FAtt.FAtt(dg, x_goal, y_goal, x_r, y_r, ka, rog)
    else:
        for i in range(len(r_pl)):
            if (np.linalg.norm(do) < limturn) or (np.dot(do, dg) <= 0):
                x_r, y_r = r_pl[i][0], r_pl[i][1] #если мы приблизились к точке поворота, то мы находимся в ней, округлим свои координаты до нее
        for i in range (0, len(short_enemy)):
            tmp = FByp.FByp(short_enemy[i], dg, x_r, y_r, x_goal, y_goal, kr, ka, ro0_pl, rog, fmax, costmap_resolution)
            print('src:', tmp, 'dst:', Fbyp)
            
            Fbyp += tmp
        u = Fbyp
        print("result bypass u = ", u)
    return u
