#!/usr/bin/env python

import math

import rospy
import tf
import tf2_ros

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from arctic_manip_service.srv import ArcticManipMoveTo,ArcticManipMoveToResponse

class ArcticManipService(object):
    def __init__(self):
        rospy.init_node('arctic_manip_service')
        # load parameters
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.manip_end_frame = rospy.get_param('~manip_end_frame', 'manip_end_link')
        self.manip_len = rospy.get_param('~manip_len', 1.)
        # inner state
        #self.rotate_pos = None
        #self.stock_pos = None
        # connect to servos
        self.rotate_pub = rospy.Publisher('manip_rotation_controller/command', Float64, queue_size=1)
        self.stock_pub = rospy.Publisher('manip_stock_controller/command', Float64, queue_size=1)
        #self.rotate_sub = rospy.Subscriber('manip_rotation_controller/state', JointControllerState, self.rotate_sub_cb)
        #self.stock_sub = rospy.Subscriber('manip_stock_controller/state', JointControllerState, self.stock_sub_cb)
        # create tf listener to get positions of the robot and the manipulator
        self.tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('map',
                                                  self.manip_end_frame,
                                                  rospy.Time(0),
                                                  rospy.Duration(1.0))
                break
            except tf2_ros.TransformException:
                rospy.logwarn('arctic_manip_service: transform from map to manipulator'
                    ' (i.e. global position of the actuator "{}") not found; waiting...'.format(self.manip_end_frame))
        
        # create service to move manipulator
        self.s = rospy.Service('manip_move_to', ArcticManipMoveTo, self.manip_move_to_cb)

    def manip_move_to_cb(self, msg):
        """ try to move manipulator to the specified point """
        # check positions of stock and base
        #if self.rotate_pos is None:
        #    rospy.logerr("arctic_manip_service: position of manupulator's base is yet unknown; can not move it")
        #    return
        #if self.stock_pos is None:
        #    rospy.logerr("arctic_manip_service: position of manupulator's stock is yet unknown; can not move it")
        #    return
        # get position of robot
        try:
            (trans,rot) = self.tf_listener.lookupTransform('map', self.robot_base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('arctic_manip_service: robot position is unknown; can not move manipulator to point')
            return
        robot_x = trans[0]
        robot_y = trans[1]
        robot_or = 2*math.atan2(rot[2], rot[3])
        # calculate position of goal as seen from the robot:
        # global pos
        vgx = msg.goal_x - robot_x
        vgy = msg.goal_y - robot_y
        # local pos = global rotated on -robot_on
        lgx = math.cos(robot_or) *vgx + math.sin(robot_or)*vgy
        lgy = -math.sin(robot_or) *vgx + math.cos(robot_or)*vgy
        # polar coordinates
        dist = (lgx**2 + lgy**2)**0.5
        angle = math.atan2(lgy,lgx)
        # calculate commands for devices
        #rospy.logerr('VG: {:.2f}, {:.2f}'.format(vgx, vgy))
        #rospy.logerr('LG: {:.2f}, {:.2f}'.format(lgx, lgy))             
        #rospy.logerr('DIST: {:.2f}, ANGLE: {:.1f}'.format(dist, angle*180 / math.pi))
        stock_cmd = 0
        if dist < 0.5*self.manip_len:
            stock_cmd = 0
        elif dist > self.manip_len:
            stock_cmd = 0.5*self.manip_len
        else:
            stock_cmd = dist - 0.5*self.manip_len
        # publish messages
        out_msg = Float64()
        out_msg.data = angle
        self.rotate_pub.publish(out_msg)
        out_msg.data = stock_cmd
        self.stock_pub.publish(out_msg)
        rospy.sleep(5.0)
        # move the manipulator in zero position
        out_msg.data = 0
        self.rotate_pub.publish(out_msg)
        self.stock_pub.publish(out_msg)
        return ArcticManipMoveToResponse(max(0, dist-self.manip_len))

    #def rotate_sub_cb(self, msg):
    #    self.rotate_pos = mgs.data

    #def stock_sub_cb(self, msg):
    #    self.stock_pos = mgs.data

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        service = ArcticManipService()
        service.run()
    except rospy.exceptions.ROSInterruptException:
        pass
