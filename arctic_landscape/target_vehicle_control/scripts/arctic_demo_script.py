#!/usr/bin/env python

""" Script for arctic demonstration that starts and stops car movement when necessary 
    and publishes car position """

import rospy
import tf
import actionlib

from std_srvs.srv import SetBool, SetBoolResponse
from signproc_soar.msg import Statement, StatementArray
from scenario_executioner.msg import StartScenarioAction, StartScenarioGoal, StopScenarioAction, StopScenarioGoal, PauseScenarioAction, PauseScenarioGoal, ResumeScenarioAction, ResumeScenarioGoal

from signproc_soar.coords_primitives import coords_to_str

class TargetDemoController(object):
    def __init__(self):
        
        # state variables
        self.car_moving = False
        self.started_once = False
        
        # scenario file
        if not rospy.has_param('~scenario_file'):
            rospy.logerr('arctic_demo: required parameter ~scenario_file not found; error')
            exit(-1)
        self.scenario_file = rospy.get_param('~scenario_file')
        
        # target scenario interfaces
        self.start_client = actionlib.SimpleActionClient('cyclic_movement_server/start_scenario',
                                                    StartScenarioAction)
        self.pause_client = actionlib.SimpleActionClient('cyclic_movement_server/pause_scenario',
                                                    PauseScenarioAction)
        self.resume_client = actionlib.SimpleActionClient('cyclic_movement_server/resume_scenario',
                                                    ResumeScenarioAction)
        
        # interface to publish car position in the semiotic subsystem
        self.tf_listener = tf.TransformListener()
        self.pub_semiotic = rospy.Publisher('/default_robot/semiotic_input', StatementArray, queue_size = 10)
        
        # timer to publish car state in semiotic system
        self.timer = rospy.Timer(rospy.Duration(1.), self.cb_timer)
        
        # server to start and stop car movement
        self.cmd_server = rospy.Service('demo_car_movement', SetBool, self.cb_car_movement)
    
    def cb_car_movement(self, req):
        """ start or pause movement """
        if self.car_moving != req.data: # skip processing if state is constant
            self.car_moving = req.data
            if self.car_moving:
                # start movement or resume movement
                if not self.started_once:
                    # start
                    self.start_client.wait_for_server()
                    goal = StartScenarioGoal(scenario_name=self.scenario_file, repeat_times=0)
                    self.start_client.send_goal(goal,
                                                done_cb=None,
                                                feedback_cb=None)
                    self.started_once = True
                else:
                    self.resume_client.wait_for_server()
                    goal = ResumeScenarioGoal()
                    self.resume_client.send_goal(goal,
                                                done_cb=None,
                                                feedback_cb=None)
            else:
                # pause scenario
                self.pause_client.wait_for_server()
                goal = PauseScenarioGoal()
                self.pause_client.send_goal(goal)
        return SetBoolResponse()
            
        
    def cb_timer(self, e):
        # read car position from tf
        try:
            (trans,rot) = self.tf_listener.lookupTransform('map', 'target_robot/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        # create output message
        s = Statement()
        s.subject = 'ki:car1'
        s.predicate = 'rdf:type'
        s.object = 'ki:car'
        s2 = Statement()
        s2.subject = 'ki:car1'
        s2.predicate = 'ki:coord'
        s2.object = coords_to_str(trans[0], trans[1])
        # send message to semiotic system
        sa = StatementArray()
        sa.data = [s,s2]
        self.pub_semiotic.publish(sa)

if __name__ == '__main__':
    try:
        rospy.init_node('signproc_translator_soar')
        tdc = TargetDemoController()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
