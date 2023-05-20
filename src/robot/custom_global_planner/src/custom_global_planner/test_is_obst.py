#!/usr/bin/env python3
import is_obstacle, obstacle_coordinate
from matplotlib import pyplot
from matplotlib.patches import *

def test_is_obst(costmap, costmap_resolution, x, y):
    a = is_obstacle.is_obstacle(costmap, costmap_resolution, x, y)
    return a
costmap_resolution = 1

costmap = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 255, 255, 255, 255, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 255, 255, 255, 255, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 0, 0, 0, 0, 0, 0], 
                                   [0, 0, 0, 255, 255, 255, 255, 255, 0, 0]]
x = 8
y = 6
print(test_is_obst(costmap, 1, x, y))

x_o, y_o = obstacle_coordinate.obstacle_coordinate(costmap)
fig = pyplot.figure()
ax = fig.add_subplot()        
for i in range(len(x_o)):
    rect = Rectangle((x_o[i], y_o[i]-costmap_resolution), costmap_resolution, costmap_resolution, color='r', alpha=0.5, zorder=2550)
    ax.add_patch(rect)
ax.plot(x, y, "go")
pyplot.xlabel('x, metres')
pyplot.ylabel('y, metres')
ax.set_aspect(aspect = "equal")
ax.set_xlim(0,10)
ax.set_ylim(0,12)
pyplot.grid()
pyplot.show()
