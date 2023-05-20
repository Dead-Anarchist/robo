#!/usr/bin/env python
# coding: utf-8

import rospy
import tf

from nav_msgs.msg import Odometry


class OdomRepublisher(object):
    def __init__(self):
        rospy.init_node('odometry_tf_republisher')
        
        # frame to use as base in published messages
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', None) # if base_frame is None frame from transformed message will be used instead
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_republish_cb)

    def odom_republish_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = []
        orientation.append(msg.pose.pose.orientation.x)
        orientation.append(msg.pose.pose.orientation.y)
        orientation.append(msg.pose.pose.orientation.z)
        orientation.append(msg.pose.pose.orientation.w)


        #rospy.logerr('odom_republisher {}: from {} to {}'.format(rospy.get_name(),
        #                                                         self.odom_frame, 
        #                                                         msg.child_frame_id 
        #                                                         if self.base_frame is None 
        #                                                         else self.base_frame))
        self.odom_broadcaster.sendTransform(
                (x, y, 0.0),
                orientation,
                msg.header.stamp,
                msg.child_frame_id if self.base_frame is None else self.base_frame,
                self.odom_frame  # NOTE: ignores msg.header.frame_id since odom must be published from odom
                        # but gazebo plugin publishes only from map
                )

    def run(self):
        rospy.spin()


def main():
    odom_repub = OdomRepublisher()
    odom_repub.run()


if __name__ == '__main__':
    main()
