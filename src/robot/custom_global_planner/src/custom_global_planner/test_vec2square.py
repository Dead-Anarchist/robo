#!/usr/bin/env python3

import math 
import numpy as np
from matplotlib import pyplot
import custom_global_planner_function as cgpf
import where_square

'''
                          1 |    2    |3
                        ____|_________|______
                            |         |
                          4 |    5    |6
                            |         |            
                        ____|_________|_____
                          7 |    8    |9
                            |         |
'''
print("#down line")
vec = cgpf.where_square.where_square(0, 3, 2, 1, 1) #down line
if np.linalg.norm(vec) == 0:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#up line")
vec = cgpf.where_square.where_square(0, 3, 2, 0.5, 3) #up line
if np.linalg.norm(vec) == 0:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()    
print("#left line")    
vec = cgpf.where_square.where_square(0, 3, 2, 0, 2) #left line
if np.linalg.norm(vec) == 0:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#right line")    
vec = cgpf.where_square.where_square(0, 3, 2, 2, 2) #right line
if np.linalg.norm(vec) == 0:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 5")    
vec = cgpf.where_square.where_square(0, 3, 2, 1, 2) #zone 5
if np.linalg.norm(vec) == 0:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 1")    
vec = cgpf.where_square.where_square(0, 3, 2, -1, 4) #zone 1
if np.linalg.norm(vec) == math.hypot(1,1):
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 2")    
vec = cgpf.where_square.where_square(0, 3, 2, 1, 5) #zone 2
if np.linalg.norm(vec) == 2:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 3")    
vec = cgpf.where_square.where_square(0, 3, 2, 3, 5) #zone 3
if np.linalg.norm(vec) == math.hypot(1,2):
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 4")    
vec = cgpf.where_square.where_square(0, 3, 2, -2, 2) #zone 4
if np.linalg.norm(vec) == 2:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 6")    
vec = cgpf.where_square.where_square(0, 3, 2, 4, 2) #zone 6
if np.linalg.norm(vec) == 2:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 7")    
vec = cgpf.where_square.where_square(0, 3, 2, -1, 0) #zone 7
if np.linalg.norm(vec) == math.hypot(1,1):
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 8")
vec = cgpf.where_square.where_square(0, 3, 2, 1, 0) #zone 8
if np.linalg.norm(vec) == 1:
    print (vec, True)
else:
    print (vec, "Incorrect norm")

print()
print("#zone 9")
vec = cgpf.where_square.where_square(0, 3, 2, 3, 0) #zone 9
if np.linalg.norm(vec) == math.hypot(1,1):
    print (vec, True)
else:
    print (vec, "Incorrect norm")


