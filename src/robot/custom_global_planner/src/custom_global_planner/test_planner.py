#!/usr/bin/env python3

import custom_global_planner_function as cgpf
import math 
import numpy as np
import obstacle_coordinate
import msg2mat
from matplotlib import pyplot
from matplotlib.patches import *

def _line_segment_intersects_square(x1, y1, x2, y2, sq_left, sq_top, sq_size):
    ''' return True if line segment from (x1,y1) to (x2, y2) intersects square 
    with top left point (sq_left, sq_top), size sq_size and horizontal orientation '''
    # other sides of the square
    sq_right = sq_left + sq_size
    sq_bottom = sq_top - sq_size
    # 1. if any end of the segment is in the square, return True
    if x1 >= sq_left and x1 <= sq_right and y1 >= sq_bottom and y1 <= sq_top:
        return True
    if x2 >= sq_left and x2 <= sq_right and y2 >= sq_bottom and y2 <= sq_top:
        return True
    # 2. if both p1 and p2 are from one side of the square, return False
    if ((x1 < sq_left and x2 < sq_left) or (x1 > sq_right and x2 > sq_right)
        or (y1 > sq_top and y2 > sq_top) or (y1 < sq_bottom and y2 < sq_bottom)):
        return False
    # 3. if segment is not vertical, calculate intersections of the segment 
    # with vertical sides
    if x1 == x2 or y1 == y2:
        # the segment is either vertical or horizontal;
        #  the first end is above/to the left from the square,
        # the other is below/to the right from the square => intersection found
        return True
    
    # calculate part of the section where the intersection
    # with vertical sides of the square situated;
    # if it is in [0, 1] and height is in square, return True
    alpha = (sq_left - x1) / (x2 - x1)
    y_l = y1 + alpha * (y2 - y1)
    if alpha >= 0. and alpha <= 1 and y_l >= sq_bottom and y_l <= sq_top:
        return True
    beta = (sq_right - x1) / (x2 - x1)
    y_r = y1 + beta * (y2 - y1)
    if beta >=0 and beta <= 1 and y_r >= sq_bottom and y_r <= sq_top:
        return True
    
    # do the same for horizontal sides of the square
    alpha = (sq_top - y1) / (y2 - y1)
    x_t = x1 + alpha * (x2 - x1)
    if alpha >= 0. and alpha <= 1 and x_t >= sq_left and x_t <= sq_right:
        return True
    beta = (sq_bottom - y1) / (y2 - y1)
    x_b = x1 + beta * (x2 - x1)
    if beta >= 0. and beta <= 1 and x_b >= sq_left and x_b <= sq_right:
        return True

    return False

assert(not _line_segment_intersects_square(0,0,1,1,3.1, 2.5, 0.3))
assert(not _line_segment_intersects_square(0,0,4,5,3.1, 2.5, 0.3))
assert(_line_segment_intersects_square(3,2.3,3.2,2.51,3.1, 2.5, 0.3))

assert(_line_segment_intersects_square(2.45598825518889,5.459952185597237,
                                       2.5218416608512326, 7.973715581857306,
                                       2, 7, 1))

def test_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y,
                    should_be_successful, eps = 0.1, skip_plot = False):
    # trying to trace a trajectory
    success, traj = cgpf.make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y, True)
    print('#'*30)
    if (should_be_successful != success):
        print('Fiels "success" is not equal to the awaited value')
        return False
    if success:
        if math.hypot(x1 - x2, y1 - y2) > eps and len(traj) == 0:
            print('The trajectory is empty')
            return False
        # check initial and final points
        if math.hypot(x1 - traj[0][0], y1 - traj[0][1]) > eps:
            print('Initial points do not match')
            return False
        if math.hypot(x2 - traj[-1][0], y2 - traj[-1][1]) > eps:
            print('Final points do not match')
            return False
        # calculate obstacles
        x_o, y_o = obstacle_coordinate.obstacle_coordinate(costmap)
        # check intersections: for each part of the trajectory check
        # if it intersects any obstacle
        for i in range(len(traj) - 1):
            start_x = traj[i][0]
            start_y = traj[i][1]
            end_x = traj[i+1][0]
            end_y = traj[i+1][1]
            for ob in range(len(x_o)):
                ob_left = x_o[ob]*costmap_resolution + costmap_origin_x
                ob_top = y_o[ob]*costmap_resolution + costmap_origin_y
                if _line_segment_intersects_square(start_x, start_y,
                                                   end_x, end_y,
                                                   ob_left, ob_top, costmap_resolution):
                    print('Trajectory intersects an obstacle')
                    return False
        #create figure
        fig = pyplot.figure()
        ax = fig.add_subplot()
        #paint trajectory
        ax.plot([v[0] for v in traj], [v[1] for v in traj],'o-')
        #paint start anf goal
        ax.plot(x1, y1, 'gD', x2, y2, 'rD')
        #paint obstacles
        print('x_o', x_o)
        print('y_o', y_o)
        print('costmap', costmap)
        print('costmap_resolution', costmap_resolution)
        print('traj', traj)
        # plot output skipping (for tests)
        if not skip_plot:
            pyplot.title("Trajectory")
            for i in range(len(x_o)):
                rect = Rectangle((x_o[i], y_o[i]-costmap_resolution), costmap_resolution, costmap_resolution, color='r', alpha=0.5, zorder=2550)
                ax.add_patch(rect)
            #rect = Rectangle((2, 2), 3, 3, color='r', alpha=0.5, zorder=2550)
        # ax.add_patch(rect)
            #paint formal text
            pyplot.xlabel('x, metres')
            pyplot.ylabel('y, metres')
            ax.set_aspect(aspect = "equal")
            # find limits
            limx_l = min(x1, x2) - 1
            limx_r = max(x1, x2) + 1
            limy_t = max(y1, y2) + 1
            limy_b = min(y1, y2) - 1
            ax.set_xlim(limx_l,limx_r)
            ax.set_ylim(limy_b,limy_t)
            pyplot.grid()
            pyplot.show()
    print('#'*30)
    return True


test_dict = {'test1' : [1, 1, 0, 6, 8, 0, 
                [
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 255, 255, 255, 255, 255],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0]
                ],
                1, 0, 0, True, 0.1, True],
                'test2' : [0, 2, 0, 4, 3, 0, 
                [[0,0,0,0], [0,0,0,0], [0,0,255, 0], [0,0,0,0]],
                1, 0, 0, True, 0.1, True],
                'test3' : [0, 2, 0, 4, 3, 0, 
                [[0,0,0,0], [0,0,255,0], [0,0,0, 0], [0,0,0,0], [0,0,0,0] ],
                1, 0, 0, True, 0.1, True],
                'test4' : [1, 1, 0, 3, 3, 0, 
                           [[0]*255]*50,
                           1, 0, 0, True, 0.1, True],
                'test5' : [1, 3.01, 0, 4, 3.01, 0, 
                           [[0,0,0,0], [0,0,0,0], [0,0,255,255], [0,0,0,0]],
                           1, 0, 0, True, 0.1, True],
                'test6' : [1, 1, 0, 4, 4, 0, 
                [
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 255, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0]
                ],
                1, 0, 0, True, 0.1, True],
                'test7' : [1, 1, 0, 6, 8, 0, 
                [
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 255, 255, 255, 255, 255],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 255, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0]
                ],
                1, 0, 0, True, 0.1, True],
                'test8' : [0, 0, 0, 2.4786670207977295, 0.3050699234008789, 0.013859708793461318, [[16, 36, 65, 149, 226, 149, 65, 36, 16, 8], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [83, 226, 254, 254, 254, 254, 254, 226, 83, 30], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [83, 226, 254, 254, 254, 254, 254, 226, 83, 30], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [16, 36, 65, 149, 226, 149, 65, 36, 16, 8], [8, 16, 36, 65, 83, 65, 36, 16, 8, 4], [4, 8, 16, 26, 30, 26, 16, 8, 4, 2], [2, 4, 7, 9, 11, 9, 7, 4, 2, 1], [1, 1, 2, 3, 4, 3, 2, 1, 1, 0]], 1.0, 0.0, 0.0, True, 0.1, True],
                'test9' : [-0.00833494606453708, -0.0038883204393804654, 0.04314161936432046, 8.315561294555664, 0.24235248565673828, 1.76673710346222, [[16, 36, 65, 149, 226, 149, 65, 36, 16, 8], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [83, 226, 254, 254, 254, 254, 254, 226, 83, 30], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [83, 226, 254, 254, 254, 254, 254, 226, 83, 30], [65, 149, 226, 254, 254, 254, 226, 149, 65, 26], [36, 65, 149, 226, 254, 226, 149, 65, 36, 16], [16, 36, 65, 149, 226, 149, 65, 36, 16, 8], [8, 16, 36, 65, 83, 65, 36, 16, 8, 4], [4, 8, 16, 26, 30, 26, 16, 8, 4, 0], [0, 4, 7, 9, 11, 9, 7, 4, 0, 0], [0, 0, 0, 0, 4, 0, 0, 0, 0, 0]], 1.0, 0.0, 0.0, True, 0.1, True],
                'test10' : [-0.004236255852779253, -0.0021321961632186507, 0.023710102352799738, 3.2873165607452393, 0.10415506362915039, -0.23082605004310608, [[30, 83, 226, 254, 254, 254, 254, 254, 226, 83], [30, 83, 226, 254, 226, 226, 226, 226, 149, 65], [30, 83, 226, 254, 226, 83, 83, 83, 65, 36], [30, 83, 226, 254, 226, 83, 83, 83, 65, 36], [30, 83, 226, 254, 226, 226, 226, 226, 149, 65], [30, 83, 226, 254, 254, 254, 254, 254, 226, 83], [30, 83, 226, 254, 226, 226, 226, 226, 149, 65], [30, 83, 226, 254, 226, 83, 83, 83, 65, 36], [30, 83, 226, 254, 226, 226, 226, 226, 149, 65], [30, 83, 226, 254, 254, 254, 254, 254, 226, 83], [26, 65, 149, 226, 226, 226, 226, 226, 149, 65], [16, 36, 65, 83, 83, 83, 83, 83, 65, 36], [8, 16, 26, 30, 30, 30, 30, 30, 26, 16], [4, 7, 9, 11, 11, 11, 11, 11, 9, 7], [0, 0, 0, 4, 4, 4, 4, 4, 0, 0]], 1.0, 0.0, 0.0, True, 0.1, True],
                
                'test11' : [-0.003598718479444577, -0.0018858418591315678, 0.020844314555592727, 10.003111839294434, 11.307409286499023, 1.6617281436920166, [[0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], 1.0, 0.0, 0.0, True, 0.1, True],
                
                'test12' : [5.651552022234806 ,  7.39737351702918 ,  2.5687290178131255 ,  1.545743465423584 ,  7.535519599914551 ,  2.920811176300049 ,  [[0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 0, 0, 0, 0, 0, 0], [0, 0, 0, 254, 254, 254, 254, 254, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]] ,  1.0 ,  0.0 ,  0.0, True, 0.1, True],
                'test13' : [0,  0,  0,  3.63899564743042 ,  3.5336558818817139 ,  3.1058104038238525 ,  [[0, 0, 0, 0], [0, 0, 254, 0], [0, 0, 0, 0], [0, 0, 0, 0]] ,  1.0 ,  0.0 ,  0.0, True, 0.1, True],
                'test14' : [0, 0, 0, 5, 1, 0,  [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
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
                                                [0, 0, 0, 255, 255, 255, 255, 255, 0, 0]], 1.0 ,  0.0 ,  0.0, True, 0.1, True],
                'test15' : [0, 0, 0, 1, 1, 0,  [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
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
                                                [0, 0, 0, 255, 255, 255, 255, 255, 0, 0]], 1.0 ,  0.0 ,  0.0, True, 0.1, False],
                'test16' : [0, 0, 0, 10, 10, 0,  [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
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
                                                [0, 0, 0, 255, 255, 255, 255, 255, 0, 0],
                                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], 1.0 ,  0.0 ,  0.0, True, 0.1, True],
                'test17' : [0, 0, 0, 9, 7, 0,  [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
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
                                                [0, 0, 0, 255, 255, 255, 255, 255, 0, 0]], 1.0 ,  0.0 ,  0.0, True, 0.1, True]}


#test_dict = {'test1' : test_dict['test1']}

# test for all examples
test_results = []
for test_name, test_params in test_dict.items():
    test_res = test_trajectory(*test_params)
    test_results.append((test_name, test_res))
    
# print all results at the end to improve readability
print('='*20)
for tn, tr in test_results:
    print(f'{tn} : {tr}')
print('='*20)



