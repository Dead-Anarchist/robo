#!/usr/bin/env python3

""" data gathering script to test the planner """

from datetime import datetime
import math
import os

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

class DataCollector:
    
    def __init__(self):
        
        # generate filenames
        start_time = datetime.now()
        base_filename = start_time.strftime('~/%Y_%m_%d_%H_%M_%S_')
        base_filename = os.path.expanduser(base_filename)
        gathered_data_list = ['cmd', 'traj', 'pos']
        self.filenames = {d : base_filename + d + '.csv'
                          for d in gathered_data_list}

        self.start_time = rospy.Time.now()
        self.curr_traj = 0
        # ROS infrastructure
        self.sub_cmd = rospy.Subscriber('mobile_base_controller/cmd_vel', Twist, self.cmd_cb)
        self.trajectory_sub = rospy.Subscriber('move_base/TebLocalPlannerROS/global_plan', Path, self.traj_cb)
        self.pos_sub = rospy.Subscriber('/gazebo_ground_truth_odom', Odometry, self.pos_cb)
        
    def cmd_cb(self, msg):
        ''' save command '''
        # calculate time from launch
        curr_moment = (rospy.Time.now() - self.start_time).to_sec()
        # open and close file for each command
        with open(self.filenames['cmd'], 'at') as f:
            # for each command save moment, linear and angular velocities
            f.write(f'{curr_moment:.4f};{msg.linear.x:.4f};{msg.angular.z:.4f}\n')

    def traj_cb(self, msg):
        ''' save trajectory '''
        # calculate time from launch
        curr_moment = (rospy.Time.now() - self.start_time).to_sec()
        # open and close file for each command
        with open(self.filenames['traj'], 'at') as f:
            # for each point of trajectory save number, current time, x, y, orientation, arriving time
            for p in msg.poses:
                x = p.pose.position.x
                y = p.pose.position.y
                ori = 2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)
                tm = p.header.stamp.to_sec()
                f.write(f'{self.curr_traj};{curr_moment:.4f};{x:.4f};{y:.4f};{ori:.4f};{tm:.4f}\n')
        self.curr_traj += 1

    def pos_cb(self, msg):
        ''' save command '''
        # calculate time from launch
        curr_moment = (rospy.Time.now() - self.start_time).to_sec()
        # open and close file for each command
        with open(self.filenames['pos'], 'at') as f:
            # for each command save moment, x, y, orientation, linear and angular velocities
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            ori = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            vx = msg.twist.twist.linear.x
            wz = msg.twist.twist.angular.z
            f.write(f'{curr_moment:.4f};{x:.4f};{y:.4f};{ori:.4f};{vx:.4f};{wz:.4f}\n')

        
if __name__ == '__main__':
    try:
        rospy.init_node('data_collector')
        dc = DataCollector()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
