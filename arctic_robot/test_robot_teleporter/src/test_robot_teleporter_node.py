#!/usr/bin/env python

import math

import rospy

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

import BaseHTTPServer as http 

class HTTPCmdHandler(http.BaseHTTPRequestHandler):
    def do_POST(self):
        self.process_data()
    def do_GET(self):
        self.process_data()
    def process_data(self):
        global ts
        l_len = int(self.headers.getheader('content-length'))
        msg = self.rfile.read(l_len)
        rospy.logerr('Input: {}'.format(msg))
        (code, ans) = ts.process_command(msg)
        self.send_response(code)
        self.end_headers()
        self.wfile.write(ans)
        
        
class TeleportServer(object):
    def __init__(self):
        
        # state variables
        self.robot_goal_pose = None
        
        # reset service
        self.reset_handler = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.teleport_handler = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.teleport_instruction = ModelState()
        self.teleport_instruction.model_name = 'arctic_model'
        self.teleport_instruction.reference_frame = 'map'
        # height to escape mesh collisions
        self.h_diff = rospy.get_param('~h_diff', 3)
        
        # subscribe on goal topics (both from action server and topic interface)
        self.sub_goal_simple = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.cb_goal_simple)
        self.sub_goal_action = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.cb_goal_action)
        # subscribe on obstacle map TODO
        
        # HTTP server
        ip_addr = rospy.get_param('~ip_address', '127.0.0.1')
        port = rospy.get_param('~port', 24001)
        self.http_server = http.HTTPServer((ip_addr, port), HTTPCmdHandler)
        self.http_timer = rospy.Timer(rospy.Duration(0.1), self.callback_http_timer)
        
        
    def cb_goal_simple(self, msg):
        # read target pose for possible "teleport" command
        self.robot_goal_pose = msg.pose

    def cb_goal_action(self, msg):
        # read target pose for possible "teleport" command
        self.robot_goal_pose = msg.goal.target_pose.pose
    
    def process_command(self, command):
        # process command from HTTP;
        # return (code, result):
        # (200, 'OK') if the command is correct;
        # (404, error message) if the command is not correct;
        command = command.strip()
        if command == 'reset':
            # restart simulation
            self.reset_handler()
            rospy.loginfo('Simulation reset completed')
        else:
            command_list = command.split(' ')
            if command_list[0].strip() == 'teleport':
                # teleportation instruction
                if len(command_list) < 5:
                    if self.robot_goal_pose is None:
                        return (404, 'goal position is unknown yet')
                    self.teleport_instruction.pose = self.robot_goal_pose
                    self.teleport_instruction.pose.position.z += self.h_diff
                else:
                    try:
                        self.teleport_instruction.pose.position.x = float(command_list[1])
                        self.teleport_instruction.pose.position.y = float(command_list[2])
                        self.teleport_instruction.pose.position.z = float(command_list[3])
                        orient = float(command_list[4])
                        self.teleport_instruction.pose.orientation.x = 0
                        self.teleport_instruction.pose.orientation.y = 0
                        self.teleport_instruction.pose.orientation.z = math.sin(orient / 2)
                        self.teleport_instruction.pose.orientation.w = math.cos(orient / 2)
                    except ValueError:
                        return (404, 'wrong format of command; correct format is "teleport x y z orientation"'.format(command))
                self.teleport_handler(self.teleport_instruction)
                rospy.loginfo('Teleportation completed')
            else:
                return (404, 'instruction "{}" not recognized'.format(command))
        return (200, 'OK')

    def callback_http_timer(self, event):
        self.http_server.handle_request()
        
if __name__ == '__main__':
    try:
        # create node
        rospy.init_node('test_robot_teleporter')
        # ts is global to use it in HTTPCommandHandler
        global ts
        ts = TeleportServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
