#!/usr/bin/env python

import copy
import math
import threading

import rospy
import actionlib

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist

import move_base_msgs.msg
import basic_movements_actions.msg

import tf

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

class BasicMovementsService(object):

    def __init__(self, control_freq,
                 mov_f_vel, mov_b_vel, min_mov_vel,
                 rot_vel, min_rot_vel):
        # create state:
        # frequency of signals to send on cmd_vel
        self.control_freq = control_freq

        # velocities
        self.mov_f_vel = mov_f_vel
        self.mov_b_vel = mov_b_vel
        self.min_mov_vel = min_mov_vel
        self.rot_vel = rot_vel
        self.min_rot_vel = min_rot_vel

        # precision
        self.mov_eps = 1e-3
        self.rot_eps = 1e-3
        # border values
        self.mov_border_val = 0.1
        self.rot_border_val = 3.0 * math.pi / 180.0

        # mutex for movement processing
        self.mutex_vel = threading.Lock()
        # is it necessary to send velocity commands
        self.direct_control = False
        self.should_stop = False

        # current position and orientation by odometry
        self.odom_x = None
        self.odom_y = None
        self.odom_or = None
        # absolute position and orientation
        self.abs_x = None
        self.abs_y = None
        self.abs_or = None
        # velocities
        self.vel_move = 0
        self.vel_rotate = 0

        # init tf
        self.robot_frame = rospy.get_param("~robot_frame","base_link")
        self.odom_frame = rospy.get_param("~odom_frame","odom")
        self.map_frame = rospy.get_param("~map_frame","map")
        self.tf_listener = tf.TransformListener()
        # check odom frame
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.robot_frame, self.odom_frame,
                                                  rospy.Time(0), rospy.Duration(2.0))
                break
            except tf.Exception:
                rospy.logwarn('{}: waiting for transform from "{}" to "{}"'.format(rospy.get_name(), self.odom_frame, self.robot_frame))
        # check map frame
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.robot_frame, self.map_frame,
                                                  rospy.Time(0), rospy.Duration(2.0))
                break
            except tf.Exception:
                rospy.logwarn('{}: waiting for transform from "{}" to "{}"'.format(rospy.get_name(), self.map_frame, self.robot_frame))

        self.movement_allowed = True
        self.reflex_sub = rospy.Subscriber('dwa_reflex/reflex_diagnostics', Bool, self.reflex_cb)
        
        # create timer for direct control for the robot
        self.timer = rospy.Timer(rospy.Duration(1.0/self.control_freq), self.timer_callback)
        
        # create timer for TF update
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.tf_timer_cb)

        # create publisher for velocity commands
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size = 1)
        # create action servers
        self.move_s = actionlib.SimpleActionServer('move_on',
                                    basic_movements_actions.msg.MoveOnAction,
                                    self.callback_move_on, 
                                    auto_start = False)
        self.rotate_s = actionlib.SimpleActionServer('rotate_on',
                                      basic_movements_actions.msg.RotateOnAction,
                                      self.callback_rotate_on, 
                                    auto_start = False)
        self.rotate_dir_s = actionlib.SimpleActionServer('rotate_dir',
                                          basic_movements_actions.msg.RotateDirAction,
                                          self.callback_rotate_dir, 
                                    auto_start = False)
        self.move_dir_s = actionlib.SimpleActionServer('move_dir',
                                                       basic_movements_actions.msg.MoveDirAction,
                                                       execute_cb=self.callback_move_dir,
                                                       auto_start = False)
        self.move_s.start()
        self.rotate_s.start()
        self.move_dir_s.start()
        self.rotate_dir_s.start()

    def reflex_cb(self, msg):
        self.movement_allowed = not msg.data

    def tf_timer_cb(self, e):
        try:
            (robot_pose,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("basic_movement_actions: can't found transform from {} to {}".format(self.odom_frame, self.robot_frame))
            return
        ''' read odometry data '''
        self.odom_x = robot_pose[0]
        self.odom_y = robot_pose[1]
        self.odom_or = 2*math.atan2(rot[2], rot[3])
        try:
            (robot_pose,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("basic_movement_actions: can't found transform from {} to {}".format(self.map_frame, self.robot_frame))
            return
        ''' read absolute position '''
        self.abs_x = robot_pose[0]
        self.abs_y = robot_pose[1]
        self.abs_or = 2*math.atan2(rot[2], rot[3])
        

    def calc_mov_velocity(self, full_dist, passed_dist, velocity_coeff = 1.):
        ''' calculate movement velocity to decrease it before finish;
        velocity_coeff allows quicken the platform (when >1) or slow it down (when >0 and <1) '''
        # 1. Return 0 if movement should be finished
        rem_dist = math.fabs(full_dist - passed_dist)
        if rem_dist < self.mov_eps:
            return 0
        # 2. Return full velocity if remaining distance is large
        if rem_dist > self.mov_border_val*velocity_coeff:
            return self.mov_f_vel*velocity_coeff if full_dist > passed_dist else self.mov_b_vel*velocity_coeff
        # 3. Return reduced velocity if remaining distance is little
        if full_dist > passed_dist:
            return (self.min_mov_vel + (full_dist - passed_dist) * (self.mov_f_vel - self.min_mov_vel) / self.mov_border_val)*velocity_coeff
        else:
            return (-self.min_mov_vel - (full_dist - passed_dist) * (self.mov_b_vel + self.min_mov_vel) / self.mov_border_val)*velocity_coeff

    def callback_move_on(self, req):
        ''' process command "move on 'dist' meters in 'max_time' seconds" '''
        # prepare result
        result = basic_movements_actions.msg.MoveOnResult()
        # process little values with no extra calls
        rospy.loginfo('move_on service: distance is {0:.3f} m'.format(req.dist))
        if math.fabs(req.dist) < self.mov_eps:
            result.success = True
            result.error_value = req.dist
            return result
        # find velocity coeff
        velocity_coeff = req.velocity_coeff if req.velocity_coeff != 0 else 1.0
        # trying to lock mutex by non-blocking call
        rt = rospy.Rate(self.control_freq)
        while not self.mutex_vel.acquire(False):
            rt.sleep()
        start_time = rospy.Time.now()
        dist_sign = 1 if req.dist > 0 else -1
        # start movement from current position
        curr_pos = (self.odom_x, self.odom_y)
        start_pos = copy.deepcopy(curr_pos)
        # estimate maximum time of movement ending
        vel = self.calc_mov_velocity(req.dist, 0, velocity_coeff = velocity_coeff)
        #vel = self.mov_f_vel if req.dist > 0 else self.mov_b_vel
        max_allowed_tm = (rospy.Time.now()
          + rospy.Duration(max(10, 2 * math.fabs(req.dist) / math.fabs(vel))))
        # start movement
        self.set_speed(vel, 0)
        rt.sleep()
        curr_dist = dist_sign * math.hypot(curr_pos[0] - start_pos[0],
                                           curr_pos[1] - start_pos[1])
        # while timeout is not reached
        while rospy.Time.now() < max_allowed_tm:
            if self.move_s.is_preempt_requested():
                # interrupt movement
                self.stop_movement()
                self.mutex_vel.release()
                result.success = False
                result.error_value = math.fabs(curr_dist - req.dist)
                self.move_s.set_succeeded(result)
                return
            if math.fabs(curr_dist) >= math.fabs(req.dist):
                # distance passed; success
                self.stop_movement()
                self.mutex_vel.release()
                result.success = True
                result.error_value = math.fabs(curr_dist - req.dist)
                self.move_s.set_succeeded(result)
                return
            rt.sleep()
            # refresh current point
            curr_pos = (self.odom_x, self.odom_y)
            curr_dist = dist_sign * math.hypot(curr_pos[0] - start_pos[0],
                                               curr_pos[1] - start_pos[1])
            # refresh velocity
            vel = self.calc_mov_velocity(req.dist, curr_dist, velocity_coeff = velocity_coeff)
            self.set_speed(vel, 0)
        # fail
        self.stop_movement()
        self.mutex_vel.release()
        result.success = False
        result.error_value = curr_dist - req.dist
        self.move_s.set_succeeded(result)

    def calc_rot_velocity(self, full_angle, passed_angle, velocity_coeff = 1.):
        ''' calculate rotation velocity to decrease it before finish '''
        # 1. Return 0 if movement should be finished
        rem_angle = full_angle - passed_angle
        if math.fabs(rem_angle) < self.rot_eps:
            return 0
        # 2. Return full velocity if remaining angle is large
        if math.fabs(rem_angle) > self.rot_border_val*velocity_coeff:
            return self.rot_vel*velocity_coeff if rem_angle > 0 else -self.rot_vel*velocity_coeff
        # 3. Return reduced velocity if remaining distance is little
        vel = (self.min_rot_vel + math.fabs(rem_angle) * (self.rot_vel - self.min_rot_vel) / self.rot_border_val)*velocity_coeff
        return vel if rem_angle > 0 else -vel

    def callback_rotate_on(self, req):
        ''' process command "move on 'dist' meters in 'max_time' seconds" '''
        # prepare result
        result = basic_movements_actions.msg.RotateOnResult()
        rospy.loginfo('rotate_on service: angle is {0:.3f} degrees'.format(req.angle * 180.0 / math.pi))

        # find velocity coeff
        velocity_coeff = req.velocity_coeff if req.velocity_coeff != 0 else 1.0

        # process little values with no extra calls
        if math.fabs(req.angle) < self.rot_eps:
            result.success = True
            result.error_value = req.angle
            return result
        # trying to lock mutex by non-blocking call
        rt = rospy.Rate(self.control_freq)
        while not self.mutex_vel.acquire(False):
            rt.sleep()
        start_time = rospy.Time.now()
        # start rotation from current orientation
        curr_or = self.odom_or
        last_or = self.odom_or
        total_rot_angle = 0
        # estimate maximum time of movement ending
        max_allowed_tm = (rospy.Time.now()
          + rospy.Duration(max(10, 2 * math.fabs(req.angle) / self.rot_vel)))
        # start rotation
        rvel = self.calc_rot_velocity(req.angle, 0, velocity_coeff = velocity_coeff)
        #rospy.logerr('Velocity: {0:.3f}'.format(rvel))
        self.set_speed(0, rvel)
        rt.sleep()
        total_rot_angle += angle_diff(curr_or, last_or)
        last_or = curr_or
        # while timeout is not reached
        while rospy.Time.now() < max_allowed_tm and self.movement_allowed:
            if self.rotate_s.is_preempt_requested():
                self.stop_movement()
                self.mutex_vel.release()
                result.success = False
                result.error_value = math.fabs(total_rot_angle) - math.fabs(req.angle)
                self.rotate_s.set_succeeded(result)
                return
            if math.fabs(total_rot_angle) >= math.fabs(req.angle):
                # distance passed; success
                self.stop_movement()
                self.mutex_vel.release()
                result.success = True
                result.error_value = math.fabs(total_rot_angle) - math.fabs(req.angle)
                self.rotate_s.set_succeeded(result)
                return
            rt.sleep()
            # refresh current point
            curr_or = self.odom_or
            total_rot_angle += angle_diff(curr_or, last_or)
            last_or = curr_or
            # refresh velocity
            rvel = self.calc_rot_velocity(req.angle, total_rot_angle, velocity_coeff = velocity_coeff)
            #rospy.logerr('Velocity: {0:.3f}'.format(rvel))
            self.set_speed(0, rvel)

        # fail
        self.stop_movement()
        self.mutex_vel.release()
        result.success = False
        result.error_value = total_rot_angle - req.angle
        self.rotate_s.set_succeeded(result)

    def timer_callback(self, timer_event):
        # message for control signals
        twist_msg = Twist()
        if self.direct_control:
            twist_msg.linear.x = self.vel_move
            twist_msg.angular.z = self.vel_rotate
            self.vel_pub.publish(twist_msg)
        elif self.should_stop:
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            self.should_stop = False

    def stop_movement(self):
        self.direct_control = False
        self.should_stop = True

    def set_speed(self, linear, angular):
        self.direct_control = True
        self.vel_move = linear
        self.vel_rotate = angular

    def rotate_to_abs_dir(self, abs_dir, service, velocity_coeff):
        rospy.logerr('Rotation to absolute direction: {}'.format(abs_dir))
        # process little values with no extra calls
        initial_err = angle_diff(abs_dir, self.abs_or)
        if math.fabs(initial_err) < self.rot_eps:
            result.success = True
            result.error_value = initial_err
            return result
        start_time = rospy.Time.now()
        # start rotation from current orientation
        curr_or = self.abs_or
        last_or = self.abs_or
        total_rot_angle = 0
        # estimate maximum time of movement ending
        max_allowed_tm = (rospy.Time.now()
          + rospy.Duration(max(10, 4 * math.fabs(initial_err) / self.rot_vel)))
        # start rotation
        rvel = self.calc_rot_velocity(initial_err, 0, velocity_coeff)
        #rospy.logerr('Velocity: {0:.3f}'.format(rvel))
        self.set_speed(0, rvel)
        rt = rospy.Rate(self.control_freq)
        rt.sleep()
        total_rot_angle += angle_diff(curr_or, last_or)
        last_or = curr_or
        # while timeout is not reached
        while rospy.Time.now() < max_allowed_tm and self.movement_allowed:
            if service.is_preempt_requested():
                return (False, angle_diff(total_rot_angle, initial_err))
            if math.fabs(total_rot_angle) >= math.fabs(initial_err):
                # distance passed; success
                self.stop_movement()
                return (True, angle_diff(total_rot_angle, initial_err))
            rt.sleep()
            # refresh current point
            curr_or = self.abs_or
            total_rot_angle += angle_diff(curr_or, last_or)
            last_or = curr_or
            # refresh velocity
            rvel = self.calc_rot_velocity(initial_err, total_rot_angle, velocity_coeff)
            #rospy.logerr('Velocity: {:.3f}, angle: {:.1f}, rem: {:.1f}'.format(
            #    rvel, total_rot_angle * 180.0 / math.pi, angle_diff(total_rot_angle, initial_err) * 180.0 /math.pi))
            self.set_speed(0, rvel)

        # fail
        self.stop_movement()
        return (False, angle_diff(total_rot_angle, initial_err))
        

    def callback_rotate_dir(self, req):
        ''' process command "rotate to direction (from east clockwise) '''
        # find velocity coeff
        velocity_coeff = req.velocity_coeff if req.velocity_coeff != 0 else 1.0
        # prepare result
        result = basic_movements_actions.msg.RotateDirResult()
        rospy.loginfo('rotate_dir service: direction is {0:.1f} degrees'.format(req.direction * 180.0 / math.pi))
        # trying to lock mutex by non-blocking call
        rt = rospy.Rate(self.control_freq)
        while not self.mutex_vel.acquire(False):
            rt.sleep()
        (result.success, result.error_value) = self.rotate_to_abs_dir(req.direction, self.rotate_dir_s, velocity_coeff)
        self.mutex_vel.release()
        self.rotate_dir_s.set_succeeded(result)

    def callback_move_dir(self, goal):
        ''' process command "rotate to direction (from east clockwise) '''
        # find velocity coeff
        velocity_coeff = goal.velocity_coeff if goal.velocity_coeff != 0 else 1.0
        rospy.loginfo('move_dir service: direction is {0:.1f} degrees'.format(goal.direction * 180.0 / math.pi))
        # trying to lock mutex by non-blocking call
        result = basic_movements_actions.msg.MoveDirResult()
        rt = rospy.Rate(self.control_freq)
        while not self.mutex_vel.acquire(False):
            rt.sleep()
        success, error_value = self.rotate_to_abs_dir(goal.direction, self.move_dir_s, velocity_coeff)
        # calculate velocity
        vel = self.calc_mov_velocity(100, 0, velocity_coeff)
        # start movement
        self.set_speed(vel, 0)
        while success:
            if rospy.is_shutdown() or self.move_dir_s.is_preempt_requested():
                self.mutex_vel.release()
                break
            rt.sleep()
        if not success:
            self.mutex_vel.release()
        result.success = success
        result.error_value = error_value
        self.move_dir_s.set_preempted(result)
        
def main():
    rospy.init_node('basic_movements_actions')

    # read required frequency of Twist signals
    try:
        control_freq = rospy.get_param("~control_freq")
    except KeyError:
        control_freq = 10.0

    # default movement and rotation velocities
    default_mov_f_vel =  0.1  # m/s
    default_mov_b_vel = -0.1  # m/s
    default_min_mov_vel = 0.01  # m/s
    default_rot_vel = 0.2  #rad/s
    default_min_rot_vel = 0.02  #rad/s

    # load velocities
    mov_f_vel = rospy.get_param("~forward_movement_velocity", default_mov_f_vel)
    if mov_f_vel < 0:
        mov_f_vel = -mov_f_vel
    elif mov_f_vel == 0:
        mov_f_vel = default_mov_f_vel

    mov_b_vel = rospy.get_param("~backward_movement_velocity", default_mov_b_vel)
    if mov_b_vel > 0:
        mov_b_vel = -mov_b_vel
    elif mov_b_vel == 0:
        mov_b_vel = default_mov_b_vel

    min_mov_vel = rospy.get_param("~min_movement_velocity", default_min_mov_vel)
    if min_mov_vel < 0:
        min_mov_vel = -min_mov_vel
    elif min_mov_vel == 0:
        min_mov_vel = default_min_mov_vel

    rot_vel = rospy.get_param("~rotation_velocity", default_rot_vel)
    if rot_vel<0:
        rot_vel = -rot_vel
    elif rot_vel == 0:
        rot_vel = default_rot_vel

    min_rot_vel = rospy.get_param("~min_rotation_velocity", default_min_rot_vel)
    if min_rot_vel<0:
        min_rot_vel = -min_rot_vel
    elif min_rot_vel == 0:
        min_rot_vel = default_min_rot_vel

    bms = BasicMovementsService(control_freq,
                                mov_f_vel, mov_b_vel, min_mov_vel,
                                rot_vel,  min_rot_vel)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
