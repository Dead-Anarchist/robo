#!/usr/bin/env python3
import numpy as np
import point_turn

enemy = [ [1, 1],
          [4, 4]
        ]
ro0_pl = 0.5
def _test_point_turn(enemy, ro0_pl):
    r_pl = point_turn.point_turn(enemy, ro0_pl)
    return(r_pl)

r_pl = _test_point_turn(enemy, ro0_pl)
print (len(r_pl), "len r_pl")
print (len(r_pl[0]), "len r_pl[0]")
for i in range (0, len(r_pl[0])):
    print ('point with coord ', i, '0 is ', r_pl[0][i]);
    print ('point with coord ', i, '1 is ', r_pl[1][i]);
    print()
    
print (r_pl)
