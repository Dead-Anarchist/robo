#!/usr/bin/env python3

import is_obstacle
import numpy as np

print('test1')

print(

)

if is_obstacle.is_obstacle([ [0,0,0,0],
                         [0,0,0,0],
                         [0,80,0,100],
                         [2,0,300,0],
                         [0,0,0,0],
                         [0,20,0,0]],
                         1, 1, 2) == True:
    print("ok")
else:
    print("AAAAAAAAAAAAAAAAAAAA")
print()

if is_obstacle.is_obstacle([ [0,0,0,0],
                         [0,0,0,0],
                         [0,80,0,100],
                         [2,0,300,0],
                         [0,0,0,0],
                         [0,20,0,0]],
                         1, 2.5, 3.7) == True:
    print("ok")
else:
    print("AAAAAAAAAAAAAAAAAAAA")
print()

if is_obstacle.is_obstacle([ [0,0,0,0],
                         [0,0,0,0],
                         [0,80,0,100],
                         [2,0,300,0],
                         [0,0,0,0],
                         [0,20,0,0]],
                         1, 1.9, 3.4) == False:
    print("ok")
else:
    print("AAAAAAAAAAAAAAAAAAAA")
print()

if is_obstacle.is_obstacle([ [0,0,0,0],
                         [0,0,0,0],
                         [0,80,0,100],
                         [2,0,300,0],
                         [0,0,0,0],
                         [0,20,0,0]],
                         1, 3.5, 2.2) == True:
    print("ok")
else:
    print("AAAAAAAAAAAAAAAAAAAA")
print()

if is_obstacle.is_obstacle([ [0,0,0,0],
                         [0,0,0,0],
                         [0,80,0,100],
                         [2,0,300,0],
                         [0,0,0,0],
                         [0,20,0,0]],
                         1, 1.5, 5.9) == False:
    print("ok")
else:
    print("AAAAAAAAAAAAAAAAAAAA")
print()
