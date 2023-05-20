#!/usr/bin/env python

import datetime
import json
import math
from enum import Enum
import os
import os.path
import threading

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates,ModelState

from cv_bridge import CvBridge

import gen_objects_description

d_dist = 1e-2
d_ori = 1.0 * math.pi / 180.0

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

def pos_are_similar(p1,p2):
    return (abs(p1[0]-p2[0]) + abs(p1[1]-p2[1]) + abs(p1[2]-p2[2]) < d_dist
            and abs(angle_diff(p1[3], p2[3])) < d_ori)

def calc_projection(dist, hfov, pix_width, pix_height, cyl_height, cyl_diam, rel_height):
    """ calculate rectangle of cylinder on camera frame;
    dist distance from camera to the cylinder;
    hfov horizontal field of view"""
    # vertical field of view
    half_hfov = hfov / 2
    half_vfov = math.asin(pix_height  * math.sin(half_hfov) / pix_width)
    # frame size on dist
    frame_x = 2 * dist * math.tan(half_hfov)
    frame_y = 2 * dist * math.tan(half_vfov)
    # positions of bottom and top rel. to center of FOV
    bottom_pix = -pix_height * rel_height / frame_y
    top_pix = bottom_pix + cyl_height/frame_y * pix_height
    # positions of left and right
    right_pix = 0.5 * cyl_diam / frame_x * pix_width
    left_pix = -right_pix
    # integer borders with limits
    top = int(top_pix + pix_height/2)
    bottom = int(bottom_pix + pix_height/2)
    if top < 0 or bottom >= pix_height:
        return []  # object is out of FOV
    top = min(top, pix_height-1)
    bottom = max(bottom, 0)
    left = int(left_pix + pix_width/2)
    right = int(right_pix + pix_width/2)
    left = max(left,0)
    right = min(right,pix_width-1)
    return [pix_height -top, pix_height -bottom, left, right]

class GathererState(Enum):
    GS_NO_ACTION = 0
    GS_TRANSFER_CAMERA = 1
    GS_GET_IMAGE = 2
    GS_SKIP_IMAGE = 3
    GS_STOP = 4

class Gatherer(object):
    def __init__(self):
        rospy.init_node('move_cam')
        self.output_dir = rospy.get_param('~output_dir')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.lock = threading.Lock()
        world_file = rospy.get_param('~world_file')
        model_library = rospy.get_param('~model_library')
        imaging_settings_file = rospy.get_param('~imaging_settings')
        use_cache_file = rospy.get_param('~use_cache_file', False)
        self.draw_debug_rects = rospy.get_param('~draw_debug_rects', False)

        self.camera_horiz_fov = rospy.get_param('~camera_horiz_fov', 1.3962634)

        self.objs = gen_objects_description.load_world(world_file, model_library, use_cache_file)

        #for k,v in self.objs.items():
        #    print(k,v)
        
        #####################################
        # !!! remove after debugging
        """
        extra_scales = {'lamp_post': 3.09 / 4.86,
                        'oak_tree': 5.52 / 217.582,
                        'house_3': 7.07 / 9.172}
        real_sizes = {'house_3': (8.42, 3.19, 7.070),
                      'oak_tree': (5.9, 3.97, 5.52),
                      'stone_large': (1.16, 1.01, 0.92),
                      'man_lay': (1.76, 0.75, 0.64),
                      'lamp_post': (0.32, 2.12, 6.18),
                      'pine_tree': (3.304, 3.17, 4.421),
                      'jersey_barrier': (4.066, 0.81, 1.143),
                      'person_walking': (0.54, 0.882, 1.872),
                      'pine_tree_broken': (1.6, 2.7, 1.58),
                      'butte': (1.4, 0.87, 1.6),
                      'set_stones': (13.9, 21, 2.017),
                      'house': (2.35, 4.000, 3.728)}
        """
        #####################################
        
        self.obj_types = {}
        for k,v in self.objs.items():
            if v[0] not in self.obj_types.keys():
                 self.obj_types[v[0]] = v[2]

        """
        # print errors
        err_lim = 0.1
        for k,v in real_sizes.items():
            ms = self.obj_types[k]
            meas_sizes = (ms[3] - ms[0], ms[4] - ms[1], ms[5] - ms[2])
            coeffs = [meas_sizes[i] / v[i] for i in range(3)]
            print('{} - {}'.format(k, coeffs))
            if any([c < 1 - err_lim or c > 1 + err_lim for c in coeffs]):
                rospy.logerr('{} - error'.format(k))
                    
        for k,v in self.obj_types.items():
            print(k,
                  ', '.join(['{:.3f}'.format(v[i+3] - v[i]) for i in range(3)]),
                  )
        """
        self.obj_cylinders = {k:(max(v[3]-v[0], v[4]-v[1]), v[5]-v[2])
                              for k,v in self.obj_types.items()}
        # shift objects in self.objs on vectors from obj_centers
        self.shift_centers_of_objects()
        
        """
        for k,v in self.obj_cylinders.items():
            print(k, ', '.join(['{:.3f}'.format(x) for x in v]))
        """
        self.load_imaging_settings(imaging_settings_file)
        self.prepare_output_file(self.output_dir)
        
        self.state = GathererState.GS_NO_ACTION if self.objs else GathererState.GS_STOP

        
        self.curr_pos = (0, 0, 0, 0)
        self.target_pos = (0, 0, 0, 0)
        self.img_counter = 0

        self.model_name = 'static_camera'

        self.pos_msg = ModelState()
        self.pos_msg.model_name = self.model_name
        self.pos_msg.reference_frame = 'world'
        
        self.gazebo_pos_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.gazebo_pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_pos_cb)

        self.camera_sub = rospy.Subscriber('/static_camera/image_raw', Image, self.camera_cb)
        self.bridge = CvBridge()

        
    def gazebo_pos_cb(self, msg):
        try:
            pos = msg.name.index(self.model_name)
        except ValueError:
            return
        x = msg.pose[pos].position.x
        y = msg.pose[pos].position.y
        z = msg.pose[pos].position.z
        ori = 2 * math.atan2(msg.pose[pos].orientation.z,
                             msg.pose[pos].orientation.w)
        self.curr_pos = (x,y,z,ori)
        # if necessary check if camera position is correct
        position_is_correct = pos_are_similar(self.curr_pos, self.target_pos)
        if (self.state == GathererState.GS_TRANSFER_CAMERA
            and position_is_correct):
            # get image
            self.state = GathererState.GS_GET_IMAGE
        elif (self.state == GathererState.GS_TRANSFER_CAMERA
              and not position_is_correct
              and self.img_counter >= 100):
            # if camera position is not correct too many times, skip it
            self.state = GathererState.GS_SKIP_IMAGE
            #rospy.logerr('Current pos is {}'.format(self.curr_pos))
        elif self.state == GathererState.GS_TRANSFER_CAMERA:
            self.img_counter += 1
#        if self.state == GathererState.GS_TRANSFER_CAMERA:
#            rospy.logerr('Pos: {}, target: {}'.format(self.curr_pos, self.target_pos))

    def camera_cb(self, msg):
        """ save image if it is necessary because of self.state """
        if self.state == GathererState.GS_GET_IMAGE:
            self.lock.acquire()
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.state = GathererState.GS_NO_ACTION
            self.lock.release()
        elif self.state == GathererState.GS_SKIP_IMAGE:
            self.lock.acquire()
            self.cv_image = None
            self.state = GathererState.GS_NO_ACTION
            self.lock.release()
            #rospy.logerr('Image gathered')

    def process_object(self, name, obj_type, obj_pos, dist):
        """ gather full set of images of point (x,y,z) from distance dist """
        rospy.loginfo('Object: {}'.format(name))
        x,y,z = obj_pos
        # generate viewpoints
        viewpoints = self.generate_viewpoints(x,y,z)

        r = rospy.Rate(10)
        # for each viewpoint
        for i, (vx,vy,vz,v_ori,d,a,h) in enumerate(viewpoints):
            # move camera to necessary point
            #rospy.loginfo('Point {} processing...'.format((vx,vy,vz,v_ori)))
            self.move_camera_to_viewpoint((vx,vy,vz,v_ori), r)
            # wait for the camera to move and get image
            while self.state != GathererState.GS_NO_ACTION:
                r.sleep()
            (rect, image_filename) = self.save_image(name, obj_type, i, d, h)
            self.add_row_to_output_file(name, obj_type, i, d, a, h, rect, image_filename)
            #rospy.logerr('Point {} achieved!'.format((vx,vy,vz,v_ori)))

    def run(self):
        r = rospy.Rate(0.2)
        r.sleep()
        # for each object in list
        for curr_obj_name,descr in self.objs.items():
            coords = descr[1][0:3]
            ori = descr[1][5] * math.pi / 180.0
            dist = 2
            self.process_object(curr_obj_name, descr[0], coords, dist)

    def generate_viewpoints(self, x,y,z):
        """ create list of viewpoints to gather photos of the object in (x,y,z)"""
        vps = []
        # for each circle
        for c in self.imaging_config:
            dist = float(c['distance'])
            height = float(c['height'])
            images_number = int(c['images_number'])
            start_angle = float(c['start_angle']) * math.pi / 180.0
            d_ori = 2.0 * math.pi / images_number
            vps.extend([(x + dist * math.cos(d_ori*i + start_angle),
                         y + dist * math.sin(d_ori*i + start_angle),
                         z + height,
                         d_ori*i + math.pi + start_angle,
                         dist,
                         d_ori*i + start_angle,
                         height) for i in range(images_number)])
        return vps

    def move_camera_to_viewpoint(self, vp, rate):
        x,y,z,ori = vp
        # prepare output message
        self.pos_msg.pose.position.x = x
        self.pos_msg.pose.position.y = y
        self.pos_msg.pose.position.z = z
        self.pos_msg.pose.orientation.z = math.sin(ori/2)
        self.pos_msg.pose.orientation.w = math.cos(ori/2)
        self.gazebo_pos_pub.publish(self.pos_msg)
        self.target_pos = (x, y, z, ori)
        rate.sleep()
        self.img_counter = 0
        self.state = GathererState.GS_TRANSFER_CAMERA
        #rospy.logerr('Camera moved')

    def save_image(self, name, obj_type, num, dist, rel_height):
        fname = '{}-{}-{}-{}.png'.format(self.date_prefix, name, obj_type, num)
        path = os.path.join(self.output_dir, fname)
        cyl = self.obj_cylinders[obj_type]

        # lock image
        self.lock.acquire()
        if self.cv_image is None:
            # image was not taken (because of obstacles or other reasons)
            self.lock.release()
            return ([-1,-1,-1,-1], 'image was not taken because of obstacles')
        height, width, channels = self.cv_image.shape
        # calculate rectangle of object
        rect = calc_projection(dist, self.camera_horiz_fov,
                               width, height,
                               cyl[1], cyl[0], rel_height)

        # draw rectangle on image
        if rect != []:
            if self.draw_debug_rects:
                top = rect[0]
                bottom = rect[1]
                left = rect[2]
                right = rect[3]
                points = [(left, top),
                          (left, bottom),
                          (right, bottom),
                          (right, top)]
                lines = [(points[i], points[i+1]) for i in range(len(points)-1)]
                lines.append((points[-1], points[0]))
                for p1,p2 in lines:
                    cv2.line(self.cv_image, p1, p2, (0,0,255), 1)
        else:
            rect = [-1, -1, -1, -1]
        cv2.imwrite(path, self.cv_image)
        self.lock.release()
        return (rect, fname)

    def load_imaging_settings(self, fname):
        with open(fname, 'rt') as f:
            data = json.load(f)
            self.imaging_config = data['movement_configuration']

    def prepare_output_file(self, images_dir):
        """ add header to csv file with images data """
        self.date_prefix = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        self.out_fname = os.path.join(images_dir, self.date_prefix + '_out.csv')
        with open(self.out_fname, 'wt') as f:
            f.write(';'.join(['model_name','model_type','image_number','distance',
                              'angle','height','top','bottom','left','right','filename'])+'\n')

    def add_row_to_output_file(self, name, obj_type, num, distance, angle, height, rect, filename):
        """ add row to csv file with data about one image """
        with open(self.out_fname, 'at') as f:
            f.write(';'.join([name, obj_type, str(num),
                              str(distance), str(angle),str(height),
                              str(rect[0]), str(rect[1]), str(rect[2]), str(rect[3]),
                              filename])+'\n')

    def shift_centers_of_objects(self):
        """ shift centers of objects to centers of cylinders' bases """
        obj_centers = {k:np.array([[(v[3]+v[0])*0.5],
                                   [(v[4]+v[1])*0.5],
                                   [v[2]]])
                       for k,v in self.obj_types.items()}
        #print('='*20)
        #print(obj_centers)
        res = {}
        #print('='*20)
        for k,v in self.objs.items():
            obj_type, pos, sizes = v
            x,y,z,R,P,Y = pos
            cY = math.cos(Y)
            sY = math.sin(Y)
            rot_m = np.array([[cY, -sY, 0],
                              [sY,  cY, 0],
                              [ 0,   0,  1]])
            #print('*'*20)
            #print(rot_m)
            #print(obj_centers[obj_type])
            #print('*'*20)
            vct = np.dot(rot_m, obj_centers[obj_type])
            #print(vct)
            res[k] = (obj_type,
                      (x + vct[0][0], y + vct[1][0], z + vct[2][0], R, P, Y),
                      sizes)
        #print('='*20)
        self.objs = res
            
            
if __name__ == '__main__':
    g = Gatherer()
    g.run()
