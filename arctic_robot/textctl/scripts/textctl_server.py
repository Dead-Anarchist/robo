#!/usr/bin/env python
import math
import copy

import http.server as http

import rospy
import tf
import actionlib
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from scenario_executioner.msg import StartScenarioAction, StartScenarioGoal

from textctl.parser import command_parser


class HTTPCmdHandler(http.BaseHTTPRequestHandler):
    def do_POST(self):
        self.process_data()
    def do_GET(self):
        self.process_data()
    def process_data(self):
        global tcs
        l_len = int(self.headers.getheader('content-length'))
        msg = self.rfile.read(l_len)
        (code, ans) = tcs.callback_http(msg)
        self.send_response(code)
        self.end_headers()
        self.wfile.write(ans)

class TextCtlServer(object):
    def __init__(self):
        self.move_precision = rospy.get_param('~move_precision', 2)      # m
        self.rotate_precision = rospy.get_param('~rotate_precision', 2)  # rad
        # connect to text input/output
        self.textpub = rospy.Publisher('textans', String, queue_size=10)
        rospy.Subscriber('textcmd', String, self.callback_cmd)
        # create http server to process commands
        #self.HTTPHandlerClass = makeHTTPCmdHandler(self)
        self.http_server = http.HTTPServer(('127.0.0.1', 23009),
                                           HTTPCmdHandler)
        http_timer = rospy.Timer(rospy.Duration(0.1), self.callback_http_timer)
        # init parser
        self.parser = command_parser
        # connect to controlling topics
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pos_pub = rospy.Publisher('move_base_simple/goal',
                                       PoseStamped, queue_size=10)
        
        self.cam_yaw_pub = rospy.Publisher('camera_yaw_controller/command',
                                           Float64, queue_size=10)
        self.cam_pitch_pub = rospy.Publisher('camera_pitch_controller/command',
                                              Float64, queue_size=10)
        self.patrol_action_client = actionlib.SimpleActionClient('start_scenario',
                                                                  StartScenarioAction)
        self.action_in_progress = False
        # init state
        self.last_cmd = ('STOP',)
        # subscribe to data topics
        # !!! TODO
        self.pos = (0, 0, 0, 0)
        self.tflistener = tf.TransformListener()
        self.world_frame = 'map'
        self.robot_frame = 'base_link'
        tr_found = False
        while not tr_found:
            try:
                self.tflistener.waitForTransform(self.world_frame, self.robot_frame,
                                                 rospy.Time(), rospy.Duration(1))
                tr_found = True
            except tf.Exception:
                rospy.logwarn('textctl_server: waiting for transform from {} to {}'.format(self.world_frame, self.robot_frame))
                pass
        rospy.Timer(rospy.Duration(0.5), self.callback_timer)
    
    def start_patrol_action(self):
        self.patrol_action_client.wait_for_server()
        goal = StartScenarioGoal(scenario_name='example', repeat_times=1)
        self.patrol_action_client.send_goal(goal,
                                  done_cb=self.callback_done_patrol_action,
                                  feedback_cb=self.callback_feedback_patrol_action)
        self.action_in_progress = True
        return '{"responce": "ok", "command": "patrol"}'

    def callback_done_patrol_action(self, terminal_state, result):
        self.action_in_progress = False
        
    def callback_feedback_patrol_action(self, feedback):
        pass
        
    def stop_patrol_action(self):
        if self.action_in_progress:
            self.patrol_action_client.cancel_goal()
            self.action_in_progress = False
        
    def callback_http_timer(self, event):
        self.http_server.handle_request()
    def callback_timer(self, event):
        try:
            (trans,rot) = self.tflistener.lookupTransform(self.world_frame,
                                                          self.robot_frame,
                                                          rospy.Time(0))
            self.pos = (trans[0],
                        trans[1],
                        rot[2],
                        rot[3])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    def callback_http(self, text):
        # parse input value
        parse_res = self.parser.parse(text)
        if parse_res is not None:
            # execute instructions
            ans_string = self.process_message(parse_res)
            res_code = 200
        else:
            ans_string = '{"responce": "error: unrecognized command", "command": "unknown"}'
            res_code = 400
        # send answer
        return (res_code, ans_string)
    def callback_cmd(self, msg):
        # parse input value
        parse_res = self.parser.parse(msg.data)
        if parse_res is not None:
            # execute instructions
            ans_string = self.process_message(parse_res)
        else:
            ans_string = '{"responce": "error: unrecognized command", "command": "unknown"}'
        # send answer
        ans = String()
        ans.data = ans_string
        self.textpub.publish(ans)
    def process_message(self, parsed_msg):
        cmd = parsed_msg[0]
        one_moment_cmds = ['CONTINUE', 'STOP', 'VELOCITY', 'TO', 'MOVE', 'ROTATE', 'AIM_CAMERA', 'GET']
        action_cmds = ['PATROL']
        if cmd in one_moment_cmds and self.action_in_progress:
            self.stop_patrol_action()
        # 1. Process instructions
        if cmd == 'CONTINUE':
            return self.process_message_continue()
        elif cmd == 'STOP':
            return self.process_msg_stop()
        elif cmd == 'PATROL':
            return self.start_patrol_action()
        elif cmd == 'VELOCITY':
            self.last_cmd = copy.copy(parsed_msg)
            return self.process_msg_velocity(float(parsed_msg[1]),
                                             float(parsed_msg[2]))
        elif cmd == 'TO':
            self.last_cmd = copy.copy(parsed_msg)
            return self.process_msg_to(float(parsed_msg[1]),
                                       float(parsed_msg[2]),
                                       float(parsed_msg[3]))
        elif cmd == 'MOVE':
            self.last_cmd = copy.copy(parsed_msg)
            return self.process_msg_move(float(parsed_msg[1]) + self.move_precision)
        elif cmd == 'ROTATE':
            self.last_cmd = copy.copy(parsed_msg)
            return self.process_msg_rotate(float(parsed_msg[1]) + self.rotate_precision)
        elif cmd == 'AIM_CAMERA':
            self.last_cmd = copy.copy(parsed_msg)
            return self.process_msg_aim_camera(float(parsed_msg[1]),
                                               float(parsed_msg[2]))
        # 2. Process getters
        elif cmd == 'GET':
            if parsed_msg[1] == 'POS':
                x = self.pos[0]
                y = self.pos[1]
                ori = 2*math.atan2(self.pos[2],self.pos[3]) * 180.0 / math.pi
                ans = '{"responce": "ok", "command":"get", "x":"'
                ans += str(x) + '", "y":"' + str(y) + '", "orientation":"'
                ans += str(ori) + '"}'
                return ans
            else:
                return '{"responce": "error: not yet supported", "command": "get"}'
        else:
            return '{"responce": "error: instruction not found", "command": "' + str(cmd) + '"}'

    def process_msg_stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_pub.publish(msg)
        return '{"responce": "ok", "command": "stop"}'
        
    def process_msg_continue(self):
        parsed_msg = self.last_cmd
        return self.process_message(parsed_msg)

    def process_msg_velocity(self, mov_vel, rot_vel):
        msg = Twist()
        msg.linear.x = mov_vel
        msg.angular.z = rot_vel*math.pi/180.0
        self.vel_pub.publish(msg)
        return '{"responce": "ok", "command": "velocity"}'

    def process_msg_move(self, mov_dist):
        curr_orient = 2*math.atan2(self.pos[2],self.pos[3])
        msg_pos = PoseStamped()
        msg_pos.header.frame_id = 'map'
        msg_pos.pose.position.x = self.pos[0] + mov_dist*math.cos(curr_orient)
        msg_pos.pose.position.y = self.pos[1] + mov_dist*math.sin(curr_orient)
        msg_pos.pose.orientation.z = self.pos[2]
        msg_pos.pose.orientation.w = self.pos[3]
        self.pos_pub.publish(msg_pos)
        return '{"responce": "ok", "command": "move"}'

    def process_msg_rotate(self, rot_angle):
        curr_orient = 2*math.atan2(self.pos[2],self.pos[3])
        msg_pos = PoseStamped()
        msg_pos.header.frame_id = 'map'
        msg_pos.pose.position.x = self.pos[0]
        msg_pos.pose.position.y = self.pos[1]
        msg_pos.pose.orientation.z = math.sin((curr_orient+rot_angle) / 2)
        msg_pos.pose.orientation.w = math.cos((curr_orient+rot_angle) / 2)
        self.pos_pub.publish(msg_pos)
        return '{"responce": "ok", "command": "rotate"}'

    def process_msg_to(self, x, y, orient):
        msg_pos = PoseStamped()
        msg_pos.header.frame_id = 'map'
        msg_pos.pose.position.x = x
        msg_pos.pose.position.y = y
        angle = orient * math.pi / 180.0
        msg_pos.pose.orientation.z = math.sin(angle / 2)
        msg_pos.pose.orientation.w = math.cos(angle / 2)
        self.pos_pub.publish(msg_pos)
        return '{"responce": "ok", "command": "to"}'

    def process_msg_aim_camera(self, pitch, yaw):
        msg_angle = Float64()
        msg_angle.data = pitch
        self.cam_pitch_pub.publish(msg_angle)
        msg_angle.data = yaw
        self.cam_yaw_pub.publish(msg_angle)
        return '{"responce": "ok", "command": "aim_camera"}'
        
    def run(self):
        rospy.spin()
        self.http_server.server_close()
    
if __name__ == '__main__':
    try:
        rospy.init_node('textctl_server')
        global tcs
        tcs = TextCtlServer()
        tcs.run()
    except rospy.ROSInterruptException:
        pass
