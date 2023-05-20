#!/usr/bin/env python3

#DU in normal form Koshi, r is r' in our task        
def func(r, time, u, T, k):
    dydt = -1/T * r + k/T * u
    return dydt
