#!/usr/bin/env python3
# coding: utf-8

"""
Autors:
    robofob/asanma(Alexander Malyshev)
    robofob/wellKnownSuperComrade

Creation Date:  2019.11.19
Last update:    2022.03.10

Script for obstacle map  generation.

It reads from *.world file 'includes' and generates obstacle map over <map_landscape>. <map_landscape> file is an high resolution image with marked all obstacles of dae model (mountains, cliffs, holes, pits, etc)

Output file is a small image (size cpecified in params)
From line:
<include><uri>model://landscape/ice</uri><name>ice1</name><pose>8 -26 0 48  49  0</pose></include>

"""

import re
import sys
from lxml import etree
import numpy
import cv2
from math import ceil
from os import path
import argparse
from pathlib import Path
import dae_processing
import math
import yaml


IMG_OUT_SIZE = (320,320) # pixels
MAP_SIZE = (160,160) # in meters

def get_obj_type(obj_str, map_config):
    """ convert resource name from model to map object
    using map_config """
    nm = obj_str.split('/')[-1]
    if nm in map_config.keys():
        return map_config[nm]
    else:
        return None

def get_descr_map(objects, start_id):
    """ print XML description of the map """
    curr_obj_id = start_id
    root = etree.Element('Map')
    # for  name, path, pos, otype, size in objects:
    for o in objects.keys():
        descr = etree.Element('Object',
                             ID = str(curr_obj_id),
                             X = str(objects[o][2][0]),
                             Y = str(objects[o][2][1]),
                             alpha = '0',
                             Name = 'Object'+str(curr_obj_id),
                             Mesh = 'package://arctic_model_gazebo/share/'+objects[o][3]+'.dae')
        root.append(descr)
        curr_obj_id += 1
    return root

def load_model_files(path):
    files =  []
    for path in Path(path).rglob('*.dae'):
        files.append(path)
    return files

def get_objects_size(objects, models, models_folder):
    no_models = []
    no_collision = []
    for o in objects:
        got_model = False
        path_to_sdf = path.join(models_folder,objects[o][1],"model.sdf")
        if path.exists(path_to_sdf):
            root = etree.parse(path_to_sdf)
            col = root.find('.//size')
            if col != None:
                size = [float(c) for c in col.text.split(' ')]
                got_model = True
            else:
                no_collision.append(o)
                for m in models:
                    if objects[o][0] == m.name.split('.')[0]:
                        got_model = True
                        size = dae_processing.get_size(m.as_posix())
                        break
        if got_model:
            objects[o].append([size[0], size[1]])
        else:
            no_models.append(o)

    if no_models != []:
        print("no models: ")
        print(no_models)

    if no_collision != []:
        print("These models have no collision defined as boxes in model.sdf:")
        print(no_collision)

    return objects

def get_pos(pos_str, outputFormat='xy'):
    """ get position from string 'x y z r p y' which is defined by outputFormat
        outputFormat:
        xyz - coords
        rpY - raw, pitch, yaw

        Note:
            in outputFormat
            lowcase y refers to y-axis values,
            uppercase Y refers to yaw
            """
    spl_data = re.sub("[^\w.,-]", " ",  pos_str).split()

    pos=()
    if 'x' in outputFormat:
        pos+=(float(spl_data[0]),)
    if 'y' in outputFormat:
        pos+=(float(spl_data[1]),)
    if 'z' in outputFormat:
        pos+=(float(spl_data[2]),)
    if 'r' in outputFormat:
        pos+=(float(spl_data[3]),)
    if 'p' in outputFormat:
        pos+=(float(spl_data[4]),)
    if 'Y' in outputFormat:
        pos+=(float(spl_data[5]),)

    if len(spl_data)<1:
        return None
    else:
        return pos

def load_world(gzworld, map_config):
    root = None
    with open(gzworld) as f :
        xml_string = f.read()
        root = etree.fromstring(xml_string)

    if root is None:
        print('Can not load data from input file')
        return -2

    objects = {}
    # for each included object
    for item in root.iter('include'):
        obj_name, obj_model_path, obj_pos, obj_type = None, None, None, None

        for subitem in item.iter('uri'):
            obj_model_path = subitem.text
            if obj_model_path is not None:
                obj_type = get_obj_type(subitem.text, map_config)
                break

        if obj_type is None:
            continue

        obj_model_path = obj_model_path.replace('model://','',1)
        obj_file_name = obj_model_path.split('/')[-1]

        for subitem in item.iter('name'):
            obj_name = subitem.text
        if obj_name is None:
            continue

        for subitem in item.iter('pose'):
            obj_pos = get_pos(subitem.text, 'xyY')
            if obj_pos is not None:
                break
        if obj_pos is None:
            continue

        objects[obj_name]= [obj_file_name, obj_model_path, obj_pos, obj_type]

    return objects

def loadImage(fileImg):
    img = cv2.imread(fileImg,cv2.IMREAD_GRAYSCALE)
    return img

def saveImage(img, fileImg, realSize=False):
    (h,w) = img.shape[:2]
    center = (w/2,h/2)
    ang=90
    scale=1
    M=cv2.getRotationMatrix2D(center, ang, scale)
    img=cv2.warpAffine(img, M, (h,w))

    if realSize:
        cv2.imwrite(fileImg,img)
    else:
        resized = cv2.resize(img, IMG_OUT_SIZE)
        cv2.imwrite(fileImg,resized)

def rotate(xy, theta):
    # https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions
    # rounded results is because of coodinates in pixels
    cos_theta, sin_theta = math.cos(theta), math.sin(theta)
    return [ round(xy[0] * cos_theta - xy[1] * sin_theta),
        round(xy[0] * sin_theta + xy[1] * cos_theta)]

def updateImage(img, pos, size):
    # !!! In images X (row) indexed with 1, Y (column) indexed with 0 !!!
    step = (MAP_SIZE[0]/float(img.shape[1]),\
            MAP_SIZE[1]/float(img.shape[0]))
    map_center = (img.shape[1]/2,\
            img.shape[0]/2)

    pos_on_map = (map_center[0]+int(round(pos[0]/step[0])), \
            map_center[1]+int(round(pos[1]/step[1])))

    size_in_pixels = (int(ceil(size[0]/(2*step[0]))),\
            int(ceil(size[1]/(2*step[1]))))

    # as there is no need draw rectangles with size of zero length
    if 0 in size_in_pixels:
        return

    # (X, Y)
    points = [ [-size_in_pixels[0],-size_in_pixels[1]],
            [size_in_pixels[0],-size_in_pixels[1]],
            [size_in_pixels[0],size_in_pixels[1]],
            [-size_in_pixels[0],size_in_pixels[1]]]

    points_rotated = []
    for p in points:
        p = rotate(p, pos[2])
        x = p[0] + pos_on_map[0]
        y = p[1] + pos_on_map[1]
        points_rotated.append([y,x])

    contour = [numpy.array(points_rotated, dtype=numpy.int32)]
    for cnt in contour:
        cv2.drawContours(img,[cnt],0,(0,0,0),-1)

def parse_config(config):
    with open(config, "r") as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--gzworld', nargs='?', help = 'gazebo world file')
    parser.add_argument('--static_map', nargs='?', default='config/arcticBasic.png', help = 'карта с нанесёнными препятствиями ландшафта и границей карты (готовится пользователями)')
    parser.add_argument('--map_config', nargs='?', default='config/map_config.yaml', help = 'mapping file')
    parser.add_argument('--models_folder', default="~/arctic_ws/src/arctic_landscape/models", nargs='?', help = 'папка с dae файлами моделей')
    parser.add_argument('--real_sized', nargs='?', type=bool, default=False, help = 'Карта препятствий не сжимается, а остаётся размера файла, указанного в параметре static_map')
    parser.add_argument('--start_id', nargs='?', type=int, default=1, help = 'начальный ID для объектов в файле-описании')
    parser.add_argument('--map_description', nargs='?', default='config/map_description.xml', help = 'генерируемый файл описания карт')
    parser.add_argument('--map_obstacles', nargs='?', default='config/map_obstacles.bmp', help = 'генерируемый файл карты препятствий')

    args = parser.parse_args()

    gzworld = args.gzworld
    static_map = args.static_map
    map_config = args.map_config
    models_folder = args.models_folder
    real_sized = args.real_sized
    start_id = args.start_id
    map_description = args.map_description
    map_obstacles = args.map_obstacles

    if not path.isfile(static_map):
        print('There\'s no static map file: '+static_map)
        exit(-1)
    if not path.isfile(gzworld):
        print('There\'s no world file: '+gzworld)
        exit(-1)

    # load param
    map_config = parse_config(map_config)

    # load data
    objects = load_world(gzworld, map_config) # подгрузка параметров вставляемых в мир объектов
    models = load_model_files(models_folder) # подгрузка dae файлов
    objects = get_objects_size(objects, models, models_folder) # определение размеров объектов
    mapBasic = loadImage(static_map) # подгрузка карты без нанесённых объектов

    for o in objects:
        name, path, pos, otype, size = objects[o]
        updateImage(mapBasic, pos, size)

    # generate description map
    root = get_descr_map(objects, start_id)

    #save maps
    saveImage(mapBasic, map_obstacles, realSize=real_sized)

    with open(map_description, 'w') as f:
        f.write( etree.tostring(root, pretty_print=True).decode('utf-8'))

