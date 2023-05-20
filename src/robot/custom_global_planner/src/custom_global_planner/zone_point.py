#!/usr/bin/env python3

#find zone, where point ~ qwadrat
def zone_point(x_sq, y_sq, a, x_r, y_r): #x_sq, y_sq - left up point of squere, a - side of squere, x_r, y_r - coordinates of robot. Result is zone where point is
    #print('Zone points : {}, {}'.format(x_r, y_r))
    zone = 0
    point_sq = [ [x_sq, y_sq], [x_sq + a, y_sq], [x_sq + a, y_sq - a], [x_sq, y_sq - a] ] #left up, right up, right down, left down points accordingly
    
    '''                   1 |    2    |3
                        ____|_________|______
                            |         |
                          4 |    5    |6
                            |         |            
                        ____|_________|_____
                          7 |    8    |9
                            |         |
    '''
 
    if (x_r <= x_sq + a) and (x_r >= x_sq) and (y_r <= y_sq) and (y_r >= y_sq - a): 
        zone = 5
    
    if (x_r <= x_sq + a) and (x_r >= x_sq):
        if y_r > y_sq: 
            zone = 2
        else: 
            zone = 8
        
    if (y_r <= y_sq) and (y_r >= y_sq - a):
        if x_r > x_sq + a: 
            zone = 6
        else: 
            zone = 4
    
    if y_r > y_sq:
        if x_r < x_sq: 
            zone = 1
        else: 
            zone = 3
            
    if y_r < y_sq - a:
        if x_r < x_sq: 
            zone = 7
        else:
            zone = 9
    
    return zone
