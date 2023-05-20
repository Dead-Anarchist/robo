#!/usr/bin/env python3
import numpy as np
import enemy_find

do_list = [ [0.15, 0.15],
          [0.14, 0.14]
        ]
ror = 0.5
limcos = 0.95
dg = [6, 6]
enemy = []
def _test_enemy_find(do_list, dg, ror, limcos):
    enemy = enemy_find.enemy_find(do_list, dg, ror, limcos)
    return(enemy)
enemy = _test_enemy_find(do_list, dg, ror, limcos)
ene = []
print(len(ene), 'len ene') #справка: если массив пустой, его длина 0
print ('enemy', enemy)
print ('len enemy', len(enemy))
print('len enemy[0]', len(enemy[0]))
