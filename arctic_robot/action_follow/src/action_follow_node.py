#!/usr/bin/env python

import math
from simple_pid import PID

import rospy
import tf

import tf2_ros

import actionlib

from geometry_msgs.msg import Twist

import action_follow.msg

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

class FollowAction(object):
    ''' action "move to the specified point smoothly enough" '''
    def __init__(self):
        
        # PID coeffs
        self.P_orient = rospy.get_param('~P_orient', 0.3)
        self.I_orient = rospy.get_param('~I_orient', 0.0)
        self.D_orient = rospy.get_param('~D_orient', 0.01)
        
        self.P_dist = rospy.get_param('~P_dist', 0.3)
        self.I_dist = rospy.get_param('~I_dist', 0.0)
        self.D_dist = rospy.get_param('~D_dist', 0.05)
        self.max_frame_skip = rospy.get_param('~max_frame_skip', 3)
        self.curr_frame_skip = 0
        self.target_found_on_frame = False
        
        self.PID_orient = PID(self.P_orient, self.I_orient, self.D_orient, setpoint=0.)
        self.PID_dist = PID(self.P_dist, self.I_dist, self.D_dist, setpoint=0.)
        self.ori_diff = 0.
        self.dist_diff = 0.
        # limits
        max_rotation_velocity = rospy.get_param('~max_rotation_velocity', 0.2)
        self.PID_orient.output_limits = (-max_rotation_velocity, max_rotation_velocity)
        max_forward_velocity = rospy.get_param('~max_forward_velocity', 0.4)
        max_backward_velocity = rospy.get_param('~max_backward_velocity', -1.)
        self.PID_dist.output_limits = (max_backward_velocity, max_forward_velocity)
        
        # base frame
        self.own_frame = rospy.get_param('~own_frame', 'base_link')
        
        
        # velocity publisher
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # action state variables
        self.motion_in_progress = False
        self.target_frame = None
        self.target_x = None
        self.target_y = None
        # command
        self.out_cmd = Twist()
            
        
        
        # timers
        self.send_commands = False
        self.empty_sent = False
        self.timer_period = rospy.get_param('~timer_period', 0.1)
        self.timer_req = rospy.Timer(rospy.Duration(self.timer_period), self.cb_timer_req)
        self.times_send = rospy.Timer(rospy.Duration(self.timer_period), self.cb_timer_send)

        # action server
        self._as = actionlib.SimpleActionServer('follow_tf_frame',
                                                action_follow.msg.FollowTFAction,
                                                execute_cb=self.cb_action,
                                                auto_start = False)
        self._as.start()
    
    def cb_timer_send(self, e):
#        rospy.logerr('=')
        if not self._as.is_active():
            self.send_commands = False

        if self.send_commands:
            self.out_cmd.angular.z = self.PID_orient(self.ori_diff)
            self.out_cmd.linear.x = self.PID_dist(self.dist_diff)
            #rospy.logerr('CMD1: {:.3f} {:.3f}'.format(self.out_cmd.linear.x, self.out_cmd.angular.z))
            self.pub_cmd_vel.publish(self.out_cmd)
        if not self.send_commands and not self.empty_sent:
            self.out_cmd.linear.x = 0.
            self.out_cmd.angular.z = 0.
            self.empty_sent = True
            #rospy.logerr('CMD2: {:.3f} {:.3f}'.format(self.out_cmd.linear.x, self.out_cmd.angular.z))
            self.pub_cmd_vel.publish(self.out_cmd)
    
    def cb_timer_req(self, e):
        # timer to process movements
        #rospy.logerr()
#        rospy.logerr('+')
        if self.motion_in_progress:
            # find current position of the goal
            try:
                #rospy.logerr('Frames: {} {}'.format(self.own_frame, self.target_frame))
                trans_and_rot = self.tf_buffer.lookup_transform(self.own_frame,
                    self.target_frame, rospy.Time.now(), rospy.Duration(2.))
                self.target_found_on_frame = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                # send zeros if large enough time interval passed
                if self.target_found_on_frame:
                    self.curr_frame_skip = 0
                else:
                    self.curr_frame_skip += 1
                self.target_found_on_frame = False
                rospy.logwarn('action follow: empty frame {}'.format(self.curr_frame_skip))
                if self.curr_frame_skip > self.max_frame_skip:
                    self.send_commands = False
                if self.curr_frame_skip == self.max_frame_skip:
                    self.empty_sent = False
                return

            # relative position of the robot
            dx = trans_and_rot.transform.translation.x
            dy = trans_and_rot.transform.translation.y
            d_orient = 2 * math.atan2(dy, dx)
            
            # difference in angle
            correct_orient = 2 * math.atan2(self.target_y, self.target_x)
            self.ori_diff = angle_diff(correct_orient, d_orient)
            
            # difference in distance
            self.dist_diff = self.target_dist - math.hypot(dx, dy)
            
            #rospy.logerr('Current distance: {:.3f}, orientation: {:.3f}'.format(math.hypot(dx,dy), d_orient))
            #rospy.logerr('Goal distance: {:.3f}, goal orient: {:.3f}'.format(self.target_dist, correct_orient))
            #rospy.logerr('Difference: by distance {:.3f}, by orient: {:.3f}'.format(self.dist_diff, self.ori_diff))
            #rospy.logerr('CMDS: vx ={:.3f}, rz={:.3f}'.format(self.out_cmd.linear.x, self.out_cmd.angular.z))
            self.send_commands = True
        else:
            self.send_commands = False
   
    def cb_action(self, goal):
        rospy.loginfo('Following started: {}'.format(goal.tf_goal))
        # move to the point specified
        res = action_follow.msg.FollowTFResult()
        
        # trying to find position of the goal frame
        while not (rospy.is_shutdown() or self._as.is_preempt_requested()):
            try:
                self.tf_buffer.lookup_transform(goal.tf_goal, self.own_frame, rospy.Time(), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn('faction follow: waiting for tf transform from "{}" to "{}"'
                              .format(goal.tf_goal, self.own_frame))
        
        # finish processing if necessary
        if rospy.is_shutdown():
            self._as.set_aborted()
            return res
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return res
        
        # start goal processing
        self.target_frame = goal.tf_goal
        self.target_x = goal.x_goal
        self.target_y = goal.y_goal
        self.target_dist = math.hypot(self.target_x, self.target_y)
        #self.PID_dist.setpoint = self.target_dist
        
        self.motion_in_progress = True
        
        # wait for either shutdown or preempt
        while not (rospy.is_shutdown() or self._as.is_preempt_requested()):
            rospy.sleep(1.)
        
        # finish processing
        if rospy.is_shutdown():
            self.motion_in_progress = False
            self._as.set_aborted()
            return res
        if self._as.is_preempt_requested():
            self.motion_in_progress = False
            self._as.set_preempted()
            return res
        
        
def main():
    try:
        rospy.init_node('action_follow_node')
        fa = FollowAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    main()
