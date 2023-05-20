#!/usr/bin/env python3
import numpy as np
import FAtt, zone_point
#count Fbyp = Frep + Ftan (Byp from word "bypass", not bypolarka!)
#Frep - power of repelling from obstacles
#Ftan - tangencial power for bypass obstacle
#do - vector-string from robot to obstacle, x_o y_o - obstacle coordinates 
#kr - koef for repelling power
#ror - radius of zone near obstacle, where potention value growing up so much, metres
#fmax - constant for ogranichenie module FByp

def FByp(do, dg, x_r, y_r, x_goal, y_goal, kr, ka, ror, rog, fmax, costmap_resolution):
    if np.linalg.norm(do) <= 2*ror and np.linalg.norm(do) > 1e-8:
        coeff = -kr * (1/np.linalg.norm(do) - 1/ror) /(np.power(np.linalg.norm(do), 3))
        print('coeff ', coeff, ', coeff')
        Frep = [v*coeff for v in do]      
        if abs(Frep[0]) > fmax:
            Frep[0]=fmax*np.sign(Frep[0])
        if abs(Frep[1]) > fmax:
            Frep[1]=fmax*np.sign(Frep[1])
    else:
        Frep = [0, 0]
            
    Fatt = FAtt.FAtt(dg, x_goal, y_goal, x_r, y_r, ka, rog)
    Ftan = [0, 0]
    if np.linalg.norm(do) > 1e-8:
        Ftan = [-do[1] * np.linalg.norm(Fatt) / np.linalg.norm(do), do[0] * np.linalg.norm(Fatt) / np.linalg.norm(do)]
        if abs(Ftan[0]) > fmax:
            Ftan[0]=fmax*np.sign(Ftan[0])
        if abs(Ftan[1]) > fmax:
            Ftan[1]=fmax*np.sign(Ftan[1])

    zone = zone_point.zone_point(do[0]+x_r, do[1]+y_r, costmap_resolution, x_r, y_r) 
       
    if (zone == 2) or (zone == 8) :
        Ftan[1] = 0
        Frep[1] = 0
    if (zone != 2) and (zone != 8):
        Ftan[0] = 0
        #Frep[0] = 0

    print("Ftan", Ftan, np.array(Ftan).shape)
    print("Frep", Frep, np.array(Frep).shape)
    print("Fatt", Fatt, Fatt.shape)
    return np.array(Ftan) + np.array(Frep) + Fatt
