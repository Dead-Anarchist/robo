#!/usr/bin/env python

import math
import xml.etree.ElementTree as ET

import rospy
import tf
from tf.transformations import quaternion_inverse, quaternion_multiply

from gazebo_msgs.msg import LinkStates

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

class JointProcessor:
    def __init__(self):
        # get robot_desctirtion from parameter
        rd = rospy.get_param('robot_description')
        self.constant_z = rospy.get_param('~constant_z', False)
        tree_root = ET.fromstring(rd)
        # calculate free joints:
        # get all joints and all joints with transmissions
        js = []
        trs = []
        self.robot_name = tree_root.attrib['name']
        for ch in tree_root:
            if ch.tag == 'joint' and ch.attrib['type']=='continuous':
                nm = ch.attrib['name']
                parent = None
                child = None
                for pr in ch:
                    if pr.tag == 'parent':
                        parent = pr.attrib['link']
                    elif pr.tag == 'child':
                        child = pr.attrib['link']
                js.append((nm,parent,child))
            elif ch.tag == 'transmission':
                for pr in ch:
                    if pr.tag == 'joint':
                        trs.append(pr.attrib['name'])
        self.free_js = [(p,ch) for (x,p,ch) in js if x not in trs]
        self.free_js_pos = {}
    def run(self):
        if self.free_js == []:
            rospy.logwarn('publish_free_joints of {}: free joints not found; nothing to do.'.format(self.robot_name))
            return
        # create broadcaster for tf transforms
        self.br = tf.TransformBroadcaster()
        self.trnsf = tf.TransformListener()
        # create timer to send data
        period = rospy.get_param('~period', 0.2)
        rospy.Timer(rospy.Duration(period), self.timer_callback)
        # subscribe to joint states from gazebo
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.ls_callback)
        rospy.spin()
    def find_link_info(self, l_name, msg):
        ind = None
        full_nm = self.robot_name + '::' + l_name
        if full_nm in msg.name:
            ind = msg.name.index(full_nm)
        if ind is None and l_name in msg.name:
            ind = msg.name.index(name)
        if ind is None:
            return (None, None)
        else:
            return ([msg.pose[ind].position.x,
                     msg.pose[ind].position.y,
                     msg.pose[ind].position.z],
                    [msg.pose[ind].orientation.x,
                     msg.pose[ind].orientation.y,
                     msg.pose[ind].orientation.z,
                     msg.pose[ind].orientation.w])
    def ls_callback(self, msg):
        # trying to get positions from gazebo messages
        for (parent, child) in self.free_js:
            # child data
            (c_p, c_o) = self.find_link_info(child, msg)
            if c_p is not None:
                self.free_js_pos[child] = (c_p, c_o)
            # parent data
            (p_p, p_o) = self.find_link_info(parent, msg)
            if p_p is not None:
                self.free_js_pos[child] = (p_p, p_o)
            
    def timer_callback(self, e):
        tm = rospy.Time.now()
        # for each free joint find parent and child in the message
        for (parent, child) in self.free_js:
            # find child position from gazebo data
            if child not in self.free_js_pos.keys():
                # if child is not in gazebo, joint can not be resolved
                rospy.logerr('Child link ({}) not found'.format(child))
                continue
            (c_p, c_o) = self.free_js_pos[child]
            # find parent position from gazebo data
            if parent in self.free_js_pos.keys():
                (p_p, p_o) = self.free_js_pos[parent]
            else:
                # if it does not found trying to get info from tf
                if self.trnsf.canTransform(parent, 'map', rospy.Time(0)):
                    (p_p, p_o) = self.trnsf.lookupTransform('map', parent, rospy.Time(0))
                else:
#                    rospy.logerr('publish_free_joints of {}: parent link {} not found in /gazebo/link_states and tf'.format(self.robot_name, parent))
                    rospy.logerr('Parent link ({}) not found'.format(parent))
                    continue
            # skip joint if any link not found in msg
            q_p_i = quaternion_inverse([p_o[0], p_o[1], p_o[2], p_o[3]])
            rot = quaternion_multiply([c_o[0], c_o[1], c_o[2], c_o[3]], q_p_i)
            disp = (c_p[0] - p_p[0], c_p[1] - p_p[1], c_p[2] - p_p[2])
            disp2 = qv_mult(q_p_i, disp)
            if self.constant_z:
                disp2[2] = 0
            self.br.sendTransform(disp2, rot,
                                  tm,
                                  child, parent)

if __name__ == '__main__':
    rospy.init_node('publish_free_joints', anonymous=True)
    jp = JointProcessor()
    jp.run()
    
