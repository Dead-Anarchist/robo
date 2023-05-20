#!/usr/bin/env python
# coding: utf-8

import copy
import csv
import os
import pickle
import re
import xml
import xml.etree.ElementTree as ET

import numpy as np
import collada
import collada.triangleset

def str_to_float(s):
    return [float(x) for x in re.split('\s+', s.strip())]

def box_to_floats(s):
    list_vals = str_to_float(s)
    res = []
    for v in list_vals:
        res.append(-v/2)
    for v2 in list_vals:
        res.append(v2/2)
    return tuple(res)

def get_filename_from_sdf(fname, curr_dir):
    """ process sdf (xml) file; get dae name """
    tree = ET.parse(fname)
    root = tree.getroot()
    #print(curr_dir.strip(' /').split('/')[-1])
    model_name = curr_dir.strip(' /').split('/')[-1]
    resources = set()
    boxes = []
    if root.tag!='sdf':
        return None
    for ch1 in root:
        if ch1.tag!='model':
            return None
        #model_name = ch1.attrib['name']
        for ch2 in ch1:
            if ch2.tag=='link':
                for ch3 in ch2:
                    if ch3.tag=='visual':
                        for ch4 in ch3:
                            if ch4.tag == 'geometry':
                                for ch5 in ch4:
                                    if ch5.tag == 'mesh':
                                        scale = (1,1,1)
                                        uri = None
                                        for ch6 in ch5:
                                            if ch6.tag == 'uri':
                                                uri = ch6.text
                                            if ch6.tag == 'scale':
                                                scale = tuple(str_to_float(ch6.text))
                                        resources.add((uri,scale))
                                    if ch5.tag == 'box':
                                        scale = (1,1,1)
                                        box = None
                                        for ch6 in ch5:
                                            if ch6.tag == 'size':
                                                box = box_to_floats(ch6.text)
                                            if ch6.tag == 'scale':
                                                scale = tuple(str_to_float_gen(ch6.text))
                                        boxes.append((box, scale))
    # get model filename from uri
    for uri,scale in resources:
        full_path =[curr_dir]
        full_path.extend(uri[uri.index('/meshes/')+1:].split('/'))
        res_fname = os.path.join(*full_path)
        box = process_dae_file(res_fname)
        if box is not None:
            boxes.append((box,scale))
        else:
            print('SDF error: model {} has wrong file "{}"'.format(model_name, res_fname))

    res = []
    if boxes == []:
        return (model_name, [])
    for i in range(6):
        lst = [b[i]*sc[i % 3] for b,sc in boxes]
        if i<3:
            res.append(min(lst))
        else:
            res.append(max(lst))
    return (model_name, res)
    

def process_dae_file(fname):
    """ get boarding box from object in dae file """
    print('Processing 3D model ' + os.path.realpath(fname))
    with open(fname, 'rb') as f:
        mesh = collada.Collada(f,
              ignore=[collada.common.DaeUnsupportedError, collada.common.DaeBrokenRefError])
        # get basic info about units
        scale = mesh.assetInfo.unitmeter
        mins = []
        maxs = []
        for n in mesh.scene.objects('geometry'):
            for pr in n.primitives():
                 mins.append(np.min(pr.vertex, axis=0))
                 maxs.append(np.max(pr.vertex, axis=0))
        if mins == [] or maxs == []:
            return None
        min_x = min([v[0] for v in mins])
        min_y = min([v[1] for v in mins])
        min_z = min([v[2] for v in mins])
        max_x = max([v[0] for v in maxs])
        max_y = max([v[1] for v in maxs])
        max_z = max([v[2] for v in maxs])
        return [min_x*scale,min_y*scale,min_z*scale,
                max_x*scale,max_y*scale,max_z*scale]
                

"""        
        # get info about scales in scenes
        scenes_matrices = {}
        for sc in mesh.scenes:
            for n in sc.nodes:
                for ob in n.objects('geometry'):
                    scenes_matrices[ob.original.id] = copy.copy(n.matrix)
        #print(scenes_matrices)
        for g in mesh.geometries:
            mesh_name = g.name + '-mesh'
            if mesh_name in scenes_matrices.keys():
                sc = scenes_matrices[mesh_name]
            else:
                sc = np.eye(4)
            for p in g.primitives:
                #print(p)
                vs = np.hstack((p.vertex, np.ones((p.vertex.shape[0],1)))).transpose()
                transp = np.matmul(sc,vs).transpose()
                mins.append(np.min(transp, axis=0))
                maxs.append(np.max(transp, axis=0))
        if mins == [] or maxs == []:
            return None
        min_x = min([v[0] for v in mins])
        min_y = min([v[1] for v in mins])
        min_z = min([v[2] for v in mins])
        max_x = max([v[0] for v in maxs])
        max_y = max([v[1] for v in maxs])
        max_z = max([v[2] for v in maxs])
        return [min_x*scale,min_y*scale,min_z*scale,
                max_x*scale,max_y*scale,max_z*scale]
"""

def get_model_type_from_uri(s):
    return s.split('/')[-1]

def process_world_file(fname):
    """ process sdf (xml) file; get dae name """
    result = {}
    tree = ET.parse(fname)
    root = tree.getroot()
    if root.tag!='sdf':
        raise ValueError('World file has wrong format: sdf tag not found')
    for ch1 in root:
        if ch1.tag!='world':
            raise ValueError('World file has wrong format: world tag not found')
        for ch2 in ch1:
            if ch2.tag=='include':
                uri = None
                pose = None
                name = None
                for ch3 in ch2:
                    if ch3.tag == 'uri':
                        uri = ch3.text
                    if ch3.tag == 'pose':
                        pose = ch3.text
                    if ch3.tag == 'name':
                        name = ch3.text
                if uri is None: 
                    raise ValueError('World file has wrong format: uri tag not found in {} {}'.
                                     format(pose, name))
                model_type = get_model_type_from_uri(uri)
                if pose is None:
                    pose = [0,0,0,0,0,0]
                else:
                    pose = [float(x) for x in re.split('\s+', pose.strip())]
                if name is None:
                    name = model_type
                result[name] = (model_type, pose)
    return result
                    

fname_sdf = 'model.sdf'
re_dae = re.compile('(.+)\.dae', re.IGNORECASE)

def load_models_library(path_to_library):
    """ process all files from directory with models """
    result = {}
    for root, dir, files in os.walk(path_to_library):
        for f in files:  # find file 'model.sdf'
            if f == fname_sdf:
                full_sdf_name = os.path.join(root,f)
                name, res = get_filename_from_sdf(full_sdf_name, root)
                result[name] = res
    return result

def calculate_models_pos(models_lib, world):
    res = {}
    for name, data in world.items():
        (model_type, pose) = data
        if model_type in models_lib.keys():
            res[name] = (model_type, pose, models_lib[model_type])
        else:
            print('Model {} not found in library; skipped.'.format(model_type))
    return res

def load_world(fname, model_library_path, use_cache=False):
    """ load description of all objects from world file"""
    # load library data from pickled file
    cache_file = fname + '_cache.bin'
    if use_cache and os.path.exists(cache_file):
        with open(cache_file,'rb') as f:
            models_lib = pickle.load(f)
    else:
        models_lib = load_models_library(model_library_path)
        inds_for_del = set()
        for k,v in models_lib.items():
            all_correct = True
            if v == []:
                print('Error; data about model {} absent'.format(k))
                inds_for_del.add(k)
                all_correct = False
        if use_cache and all_correct:
            with open(cache_file,'wb') as f:
                pickle.dump(models_lib, f)
        for ind in inds_for_del:
            del models_lib[ind]

    # process world file
    world = process_world_file(fname) #'../src/arctic_model/arctic_model_gazebo/worlds/arctic3.world')
    res = calculate_models_pos(models_lib, world)
    return res

    
#with open('object_descriptions.csv','wt') as f:
#    writer = csv.writer(f, delimiter=';')
#    writer.writerow(['object','type','x','y','z','roll','pitch','yaw','min_x','min_y','min_z','max_x','max_y','max_z'])
#    for k,v in res.items():
#        lst = [k, v[0]]
#        lst.extend(v[1])
#        lst.extend(v[2])
#        writer.writerow(lst)
