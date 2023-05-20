#!//usr/bin/env python
# coding: utf-8

"""
Autors:
    robofob/asanma(Alexander Malyshev)

Creation Date:  2022.01.28
Last update:    2022.03.10

"""

import numpy as np
from collada import *
import argparse
from os import path

def get_size(dae_file, unit_correction=0.001):
    object_size = None
    if not path.isfile(dae_file):
        print('No such file: '+ dae_file)
    else:
        try:
            dae_mesh = Collada(dae_file)
        except:
            print("some problems in file: " + dae_file)
            print("cannot define size")
            return object_size
        primitives = dae_mesh.geometries[0].primitives[0]
        faces = np.array(np.expand_dims(primitives[0].vertices, axis=0))
        for i in range(1,len(primitives)):
            f = np.array(np.expand_dims(primitives[i].vertices, axis=0))
            faces = np.append(faces,f,axis=0) 

        max_cood = np.amax(faces, axis=0)
        max_cood = np.amax(max_cood, axis=0)
        min_coord = np.amin(np.amin(faces, axis=0), axis=0)
        # print(max_cood)
        # print(min_coord)

        object_size = (max_cood-min_coord)*unit_correction
        # print("object size, m:")
        # print(object_size)

    return object_size

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dae_file', nargs='?', help = 'dae file to define object size')

    args = parser.parse_args()

    dae_file = args.dae_file
    print(get_size(dae_file, 1))
