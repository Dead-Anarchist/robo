#include<algorithm>
#include<cmath>

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

//#include "msg_rsaction/action.h"
//#include "msg_odom/Encoders.h"

//#include "rcproto/tmucmd.h"
//#include "rcproto/rcproto2.h"
//#include "i2cpro/i2cwctl.h"

//using std::map;

class RobotDriver{
 private:
  // robot parameters (fixed after initialization)
  double wheel_dist;
  double wheel_radius;
  std::string lwheel_joint;
  std::string rwheel_joint;
  double l_speed;  // rad/s
  double r_speed;  // rad/s

  // ROS infrastructure
  ros::Publisher lwheel_pub;
  ros::Publisher rwheel_pub;
  ros::Publisher odom_pub;
  double vel_update_rate;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber joints_sub;
  ros::Timer timer;

  // robot state (variables)
  // wheel positions
  double pos_l; 
  double pos_r;
  // robot coordinates
  double robot_x; 
  double robot_y;
  // robot orientation
  double robot_or;
  
  // auxiliary methods
  void load_parameters();
  void setup_infrastructure();
  void update_position(double new_pos_l, double new_pos_r);
  // callbacks
  void update_velocity_cb(const ros::TimerEvent&);
  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr & msg);
  void joint_states_cb(const sensor_msgs::JointState::ConstPtr msg);


 public:
  RobotDriver() : l_speed(0), r_speed(0), robot_x(0), robot_y(0), robot_or(0) { }
  void init() {
    load_parameters();
    setup_infrastructure();
  }
};

// send commands to gazebo controllers
void RobotDriver::update_velocity_cb(const ros::TimerEvent&) {
  std_msgs::Float64 out_msg;
  out_msg.data = l_speed;
  lwheel_pub.publish(out_msg);
  out_msg.data = r_speed;
  rwheel_pub.publish(out_msg);
}

// process velocity commands
void RobotDriver::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr & msg) {
  double g_vel = msg->linear.x; // m/s
  double t_vel = msg->angular.z; // rad/s

  ROS_DEBUG("cmd_vel raw velocity command: lin, ang=[%lf, %lf]", msg->linear.x, msg->angular.z);
  
  l_speed = g_vel/wheel_radius - (t_vel * wheel_dist) / (2 * wheel_radius);
  r_speed = g_vel/wheel_radius + (t_vel * wheel_dist) / (2 * wheel_radius);
  
  ROS_DEBUG("cmd_vel processed velocity command: lwheel, wheel=[%lf, %lf]", l_speed, r_speed);
}

void RobotDriver::joint_states_cb(const sensor_msgs::JointState::ConstPtr msg){
  double new_pos_l, new_pos_r;
  // find position of left wheel in vectors
  auto it_f = std::find(msg->name.begin(),
                        msg->name.end(),
                        lwheel_joint);
  if (it_f != msg->name.end()) {
    // calculate state of the left wheel
    new_pos_l = msg->position[it_f - msg->name.begin()];
  } else {
    return;
  }
  // do the same with the right wheel
  it_f = std::find(msg->name.begin(),
                   msg->name.end(),
                   rwheel_joint);
  if (it_f != msg->name.end()) {
    new_pos_r = msg->position[it_f - msg->name.begin()];
  } else {
    return;
  }
  // update odometry data by new positions of wheels
  update_position(new_pos_l, new_pos_r);
  // send results to odom topic
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "map";
  //odom_msg.child_frame_id = ros::this_node::getNamespace() + "base_link";
  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.orientation.z = sin(robot_or/2);
  odom_msg.pose.pose.orientation.w = cos(robot_or/2);
  odom_pub.publish(odom_msg);
}

void RobotDriver::update_position(double new_pos_l, double new_pos_r){
  // calculate orientation vector of the robot
  double dir_x = cos(robot_or);
  double dir_y = sin(robot_or);
  // calculate shifts of wheels from the last positions
  double shift_l = (new_pos_l - pos_l) * wheel_radius;
  double shift_r = (new_pos_r - pos_r) * wheel_radius;
  // update positions of wheels
  pos_l = new_pos_l;
  pos_r = new_pos_r;
  // if the shifts are similar, move the robot without rotation
  if (fabs(shift_l - shift_r) < 1e-6) {
    robot_x += dir_x * shift_l;
    robot_y += dir_y * shift_l;
    return;
  }
  // the shifts are different => the trajectory is arc.
  double delta_or = (shift_r - shift_l) / wheel_dist;
  double c = wheel_dist * 0.5 * (shift_l + shift_r) / (shift_l - shift_r);
  double delta_pos_x = - c * sin(delta_or);
  double delta_pos_y = - c * (1 - cos(delta_or));
  // update coordinates and orientation of the robot
  robot_x += delta_pos_x * dir_x - delta_pos_y * dir_y;
  robot_y += delta_pos_x * dir_y + delta_pos_y * dir_x;
  robot_or = fmod(robot_or + delta_or, 2 * M_PI);
}


void RobotDriver::load_parameters(){
  ros::NodeHandle private_nh("~");
  
  if (private_nh.hasParam("wheel_dist")){
    private_nh.getParam("wheel_dist", wheel_dist);
  } else {
    ROS_INFO("Parameter wheel_dist not found in %s; set to 1",
	     ros::this_node::getName().c_str());
    wheel_dist = 1;
  }
  if (private_nh.hasParam("wheel_radius")){
    private_nh.getParam("wheel_radius", wheel_radius);
  } else {
    ROS_INFO("Parameter wheel_radius not found in %s; set to 0.3",
	     ros::this_node::getName().c_str());
    wheel_radius = 0.3;
  }
  if (private_nh.hasParam("lwheel_joint")){
      private_nh.getParam("lwheel_joint", lwheel_joint);
  } else {
    ROS_INFO("Parameter lwheel_joint not found in %s; set to lwheel_j",
                ros::this_node::getName().c_str());
    lwheel_joint = "lwheel_j";
  }
  if (private_nh.hasParam("rwheel_joint")){
      private_nh.getParam("rwheel_joint", rwheel_joint);
  } else {
    ROS_INFO("Parameter rwheel_joint not found in %s; set to rwheel_j",
	      ros::this_node::getName().c_str());
    rwheel_joint = "rwheel_j";
  }
  
  private_nh.param<double>("vel_update_rate", vel_update_rate, 20);
  private_nh.param<double>("start_x", robot_x, 0);
  private_nh.param<double>("start_y", robot_y, 0);
  private_nh.param<double>("start_or", robot_or, 0);
}

void RobotDriver::setup_infrastructure() {
  ros::NodeHandle nh;
  
  // publishers for velocities
  lwheel_pub = nh.advertise<std_msgs::Float64>("lwheel_state_controller/command", 1);
  rwheel_pub = nh.advertise<std_msgs::Float64>("rwheel_state_controller/command", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
        
  // subscriber for speed commands
  cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RobotDriver::cmd_vel_cb, this);

  // subscriber for joints descriptions
  joints_sub = nh.subscribe("joint_states", 1, &RobotDriver::joint_states_cb, this);

  // timer for velocity processing
  timer = nh.createTimer(ros::Duration(1 / vel_update_rate),
			 &RobotDriver::update_velocity_cb, this);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_robot_driver");
  RobotDriver dr;
  dr.init();
   
  ros::spin();
  
  return 0;
}
