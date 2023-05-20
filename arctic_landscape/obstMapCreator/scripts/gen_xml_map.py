#!//usr/bin/env python
# coding: utf-8

"""
Script for map file generation from Gazebo world model.

It reads from *.world file 'includes' and generates from each an object for map using known relations between Gazebo and map objects.
From line:
<include><uri>model://landscape/ice</uri><name>ice1</name><pose>8 -26 0 48  49  0</pose></include>

it generates:
<Object ID="1" X="9" Y="-26" alpha="0" Name="Rock1" Mesh="package://arctic_model_gazebo/share/rock.dae"/>

because an ice pack from Gazebo is a rock in map.

The script should be used to supplement map file, not generate it fully because some map objects (pits) are not described in Gazebo map directly.
"""

import re
import sys
from lxml import etree
import argparse
import yaml


def get_obj_type(obj_str, map_config):
    """ convert resource name from model to map object 
    using map_config """
    nm = obj_str.split('/')[-1]
    if nm in map_config.keys():
        return map_config[nm]
    else:
        return None

def get_pos(pos_str):
    """ get position as (x,y) from string 'x y z r p y' """
    spl_data = re.sub("[^\w.,-]", " ",  pos_str).split()
    if len(spl_data)<2:
        return None
    else:
        return (float(spl_data[0]), float(spl_data[1]))

def load_model(input_filename, map_config):
    root = None
    # read data
    with open(input_filename) as f :
        xml_string = f.read()
        root = etree.fromstring(xml_string)
    if root is None:
        print('Can not load data from input file')
        return -2
    obj_arr = []
    # for each included object
    for item in root.iter('include'):
        obj_type, obj_pos = None, None
        # get type
        for subitem in item.iter('uri'):
            obj_type = get_obj_type(subitem.text, map_config)
            if obj_type is not None:
                break
        # go to next if type not found
        if obj_type is None:
            continue
        for subitem in item.iter('pose'):
            obj_pos = get_pos(subitem.text)
            if obj_pos is not None:
                break
        if obj_pos is None:
            continue
        obj_arr.append((obj_type, obj_pos))
    return obj_arr

def gen_XML_string(obj_type, obj_pos, curr_obj_id):
    obj = etree.Element('Object',
                         ID = str(curr_obj_id),
                         X = str(obj_pos[0]),
                         Y = str(obj_pos[1]),
                         alpha = '0',
                         Name = 'Object'+str(curr_obj_id),
                         Mesh = 'package://arctic_model_gazebo/share/'+obj_type+'.dae')
    return etree.tostring(obj)

def get_map(gzworld, start_id):
    """ print XML description of the map """
    curr_obj_id = start_id
    root = etree.Element('Map')
    for obj_type, obj_pos in gzworld:
        # print(gen_XML_string(obj_type, obj_pos, curr_obj_id))
        obj = etree.Element('Object',
                             ID = str(curr_obj_id),
                             X = str(obj_pos[0]),
                             Y = str(obj_pos[1]),
                             alpha = '0',
                             Name = 'Object'+str(curr_obj_id),
                             Mesh = 'package://arctic_model_gazebo/share/'+obj_type+'.dae')
        root.append(obj)
        curr_obj_id += 1
    return root

def parse_config(config):
    with open(config, "r") as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gzworld', nargs='?', help = 'gazebo world file')
    parser.add_argument('--start_id', nargs='?', type=int, default=1, help = 'gazebo world file')
    parser.add_argument('--map_config', nargs='?', default='config/map_config.yaml', help = 'mapping file')
    parser.add_argument('--map_description', nargs='?', default='config/map_description.xml', help = 'генерируемый файл описания карт')

    args = parser.parse_args()
    gzworld = args.gzworld
    start_id = args.start_id
    map_config = args.map_config
    map_description = args.map_description

    # load param
    map_config = parse_config(map_config)

    # load data
    gzworld = load_model(gzworld, map_config)

    # generate and save description map 
    root = get_map(gzworld, start_id)
    with open(map_description, 'w') as f:
        f.write( etree.tostring(root, pretty_print=True).decode('utf-8'))
