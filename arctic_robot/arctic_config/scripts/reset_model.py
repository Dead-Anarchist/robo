#!/usr/bin/env python
#coding:utf-8

import rospy

from gazebo_msgs.srv import SetModelState, SetModelStateRequest

import sys
import logging

log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)
formatter = logging.Formatter(fmt='%(levelname)s %(module)s:\n%(message)s')

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
log.addHandler(ch)


def reset_model():
    set_model_state= rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    srv_args = SetModelStateRequest()
    srv_args.model_state.model_name = 'arctic_model'

    srv_args.model_state.pose.position.x = 0.
    srv_args.model_state.pose.position.y = 0.
    srv_args.model_state.pose.position.z = 0.5

    srv_args.model_state.pose.orientation.x = 0.
    srv_args.model_state.pose.orientation.y = 0.
    srv_args.model_state.pose.orientation.z = 1.
    srv_args.model_state.pose.orientation.w = 1.

    srv_args.model_state.twist.linear.x = 0.
    srv_args.model_state.twist.linear.y = 0.
    srv_args.model_state.twist.linear.z = 0.

    srv_args.model_state.twist.angular.x = 0.
    srv_args.model_state.twist.angular.y = 0.
    srv_args.model_state.twist.angular.z = 0.

    response = set_model_state(srv_args)

    log.info(str(response))


def main():
    reset_model()


if __name__ == '__main__':
    main()
