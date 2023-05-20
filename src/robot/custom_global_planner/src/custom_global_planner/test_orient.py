#!/usr/bin/env python3
import numpy as np
import orient
from matplotlib import pyplot
def test_orient(xs, ys, ori_start, ori_end):
    ori = orient.orient(xs, ys, ori_start, ori_end)
    for i in range(len(ori)):
        ori[i] = ori[i] * 180 / np.pi
        print(ori[i], '/n')
    #create figure
    fig = pyplot.figure()
    ax = fig.add_subplot()
    #paint trajectory
    ax.plot(xs, ys, 'o-')
    for i in range(len(ori)):
        
        ax.annotate(str(ori[i]), xy = (xs[i], ys[i]), xytext=(xs[i]-0.2, ys[i]-0.2))
    #paint start anf goal
    ax.plot(xs[0], ys[0], 'gD', xs[-1], ys[-1], 'rD')
    pyplot.title("test traj and angle")
    pyplot.grid()
    pyplot.show()    

xs = [0,1,1,2,3,4,4,5]
ys = [0,0,1,1,2,1,0,1]
ori_start = 0
ori_end = 0
test_orient(xs, ys, ori_start, ori_end)
