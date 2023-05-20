#!/usr/bin/env python3

import numpy as np
from scipy.integrate import odeint
from scipy import integrate
import FAtt, FByp, bypass, obstacle_coordinate, min_skal, du, zone_point, where_square, new_point, is_obstacle, enemy_find, obst, point_turn, orient

"""
For setting our control system we use come constants
FAtt.py - function with attractive power: 
- **dg** - vector between robot and goal
- **rog** - radius of zone near goal, where we can use parabolic potention field
- **ka** - koefficient attractive power

FByp.py - bypass power = Frep + Ftan
- **do** - vector between robot and obstacle
- **kr** - koef repelling power
- **ror** - radius of zone near obstacle, where potention value growing up so much
- **fmax** - max module for FByp

bypass.py - function for bypass

obstacle_coordinate.py - parsing costmap for find obstacles

min_skal.py - output is minimal vector to nearest obstacle

du.py - transmit du for normal form Koshi

zone_point.py - function for relayshion between coordinates and gazebo net (qwadrats with costmap)

where_square.py - find vector from point to qwadrat

new_point.py - for solve DU

is_obstacle.py - answer the question "is this point obstacle or not?"

enemy_find - func for search nearest obstacles

obst - find obstacles in half of cicle on our way

point_turn - поиск точек поворота

orient - for good trajectory, find orientation and every point of trajectory
"""
def make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y, debug_mode = False):
    try:
        return _make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y)
    except Exception as e:
        if debug_mode:
            raise e
        else:
            print(e)
            return (False, [])
    except:
        if debug_mode:
            raise e
        else:
            print('unknown error')
            return (False, [])
    
def _make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y):
    
    print(x1, ', ', y1, ', ', ori1, ', ',
                    x2, ', ', y2, ', ', ori2,', ',
                    costmap, ', ', costmap_resolution, ', ',
                    costmap_origin_x, ', ', costmap_origin_y) 
    
    x_o, y_o = obstacle_coordinate.obstacle_coordinate(costmap)
   
    kr = 0.12 #for repelling power
    ka = 0.6  #for attractive power
   
   #ror - radius of zone near obstacle, where potention value growing up so much
    ror = 0.5 #metres
    #if there are no obstacles in radius ro0_pl, we have free moving, ro0_pl > ror - width of zone of obstacle influence, ro0_pl=2*ror
    ro0_pl = 2*ror #metres
    rog = 0.3 #radius of zone, where we think, that goal is roached
    
    #limcos - limit of cos angle between do and dg, when we count FByp in bypass module. 
    #if we see goal, but obstacle behind it, we mustn't afraid of obstacle.
    limcos = 0.95
    #limturn - limit different between robot position and point of turn
    limturn = 0.1
    #constant for ogranichenie module FByp
    fmax = 2 
    
    #constants for DU Tr'' + r' = ku, u - control impact. u = Fatt, if we have free moving mode
    #u = Ftan + Frep, if we have bypass mode
    T = 0.2 
    k = 1
        
    #x_r y_r - coordinates of robot's position
    x_r = x1
    y_r = y1
    #x_goal y_goal - coordinates of end (goal) robot's position 
    x_goal = x2
    y_goal = y2
    #initial conditions for robot
    x_start = x1 
    y_start = y1

    #new_msg - output of this function - new massage with list of points for way from start to goal for ROS
    out_x = []
    out_y = []
    out_x.append(x_start) #app start pozition
    out_y.append(y_start)        
        
    #let t (time argument) change in range 0 to 10 sec, 10 points
    t = np.linspace(0, 1, 10)
        
    #we have 3 variables of system condition. 
    #1) robot can move free - free mode, u = Fatt
    #2) robot bypass obstacle - bypass mode, u = Fbyp
    #3) robot comes goal

    dg = np.array([x_goal - x_r, y_goal - y_r])
    do_list = []
    for i in range (0, len(x_o)):
        do_list.append([x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r])
    short_do_list = obst.dofind(do_list, x_r, y_r, dg, ro0_pl, costmap_resolution)
    do = []
    #find important obstacles and points of turn
    short_enemy = enemy_find.enemy_find(short_do_list, dg, ro0_pl, ro0_pl)
    if len(short_enemy) > 0:
        r_pl =  point_turn.point_turn(short_enemy, ro0_pl)
        do = min_skal.min_skal(short_enemy, costmap_resolution) #do - кратчайшее расстояние до препятствия, short_enemy - список интересующих нас перпендикуляров до препятсствий
    print('do important: ', do)
    iter_ = 0 #need for break when we test it and prog work so long
    
    #speeds for DU
    velocity_x = []
    velocity_x.append(0)
    velocity_y = []
    velocity_y.append(0)
    
    while np.linalg.norm(dg) > rog: #let comfort distance to goal is rog = 30 cm
        iter_ += 1
        if iter_ > 200:
            break

        print(dg, 'dg')
        print(do, 'do')
        
        reserve_dg = dg #save previos dg for backstep in case of coming in zone 5
        reserve_do = do #save previos do for backstep in case of coming in zone 5
        reserve_rob = [x_r, y_r] #save previos koordinates of robot for backstep in case of coming in zone 5
        reserve_short_enemy = short_enemy
        if len(do) > 0 and np.dot(dg, do) > 0 and np.linalg.norm(do) < ro0_pl and np.linalg.norm(do) > 1e-8: 
            u = bypass.bypass(short_enemy, do, dg,
                              costmap_resolution, x_r, y_r,
                              x_goal, y_goal, 
                              ka, kr, rog, ror, ro0_pl,
                              fmax, limcos, limturn)    
        else:
            u = FAtt.FAtt(dg, x_goal, y_goal, x_r, y_r, ka, rog)
            print(u, "no onbstacle")
        x_r, y_r = new_point.new_point(velocity_x, velocity_y, x_r, y_r, t, u, T, k)
        print("Iteration number", iter_, ":", x_r, "x_r after while", y_r, "y_r after while")
        out_x.append(x_r)
        out_y.append(y_r)
        dg = np.array([x_goal - x_r, y_goal - y_r])
        
        #found obstacles in new position
        for i in range (0, len(x_o)):
            do_list[i] = [x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r]
        print("do_list in new pos", do_list)
        short_do_list = obst.dofind(do_list, x_r, y_r, dg, ro0_pl, costmap_resolution)
        print("short_do_list in new pos", short_do_list)
        short_enemy = enemy_find.enemy_find(short_do_list, dg, ro0_pl, ro0_pl)
        print("enemy in new pos", short_enemy)
        
        if len(short_enemy) > 0:
            r_pl =  point_turn.point_turn(short_enemy, ro0_pl)
            do = min_skal.min_skal(short_enemy, costmap_resolution)
        else: 
            do = []
            r_pl = []

        print("do after while", do)
        
        #recount if new point is obstackle
        koef = 0.1
        test_on_obst_bool = [is_obstacle.is_obstacle(costmap, costmap_resolution, x_r, y_r, ror)]
        ind = 0
        tankoef = y_r/x_r
        while (is_obstacle.is_obstacle(costmap, costmap_resolution, x_r, y_r, ror)):
            
            ind += 1
            if ind > 20: 
                break
            test_on_obst_bool.append(is_obstacle.is_obstacle(costmap, costmap_resolution, x_r, y_r, ror))
            do = reserve_do
            dg = reserve_dg
            short_enemy = reserve_short_enemy
            short_do_list = obst.dofind(do_list, x_r, y_r, dg, ror, costmap_resolution)
            
            if np.linalg.norm(do) <= ro0_pl:
                u = bypass.bypass(short_enemy, do, dg, 
                                  costmap_resolution, x_r, y_r, 
                                  x_goal, y_goal, ka, kr, 
                                  rog, ror, ro0_pl, 
                                  fmax, limcos, limturn)
            x_r = reserve_rob[0]
            y_r = reserve_rob[1]
            
            xx_r, yy_r = new_point.new_point(velocity_x, velocity_y, x_r, y_r, t, u, T, k)
            
            x_r = xx_r - koef*np.cos(np.arctan2(yy_r - y_r, xx_r - x_r))
            y_r = yy_r - koef*np.sin(np.arctan2(yy_r - y_r, xx_r - x_r))
            koef+=0.1
            print(x_r, "x_r PROCH", y_r, "y_r PROCH")
            out_x[-1] = x_r
            out_y[-1] = y_r
        print('test_on_obst_bool', test_on_obst_bool)
        
        
        #check_goal()         
        if (np.linalg.norm(dg) > rog) and (np.dot([x_goal - out_x[-1], y_goal - out_y[-1]], [x_goal - out_x[-2], y_goal - out_y[-2]]) > 0):
            out_x.append(x_r)
            out_y.append(y_r)
        elif np.dot([x_goal - out_x[-1], y_goal - out_y[-1]], [x_goal - out_x[-2], y_goal - out_y[-2]]) < 0:
            x_r = reserve_rob[0]
            y_r = reserve_rob[1]
            out_x[-1] = x_r
            out_y[-1] = y_r
        print(dg, "dg after while")
        print()
        print()
        
    out_x.append(x_goal)
    out_y.append(y_goal)    
    
    out_ori = orient.orient(out_x, out_y, ori1, ori2)
    
    output_list = [(out_x[i], out_y[i], out_ori[i])
        for i in range(len(out_x))]
    
    print()
    print(output_list, 'output')
    print('we work, count of iter = ', iter_)
    return (True, output_list)    
