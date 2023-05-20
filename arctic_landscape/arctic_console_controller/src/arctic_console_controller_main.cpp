#include<cstdio>
#include<string>

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<utility>
#include<vector>
#include"getch.h"

using std::vector;

int main(int argc, char **argv){
  // initialize node
  ros::init(argc, argv, "arctic_console_controller");
  // initialize publishing to the topic /ldtester_linecontroller/manual_direction
  ros::NodeHandle n;
  ros::Publisher r_pub, l_pub; 
  r_pub = n.advertise<std_msgs::Float64>("/rwheel_state_controller/command", 50);
  l_pub = n.advertise<std_msgs::Float64>("/lwheel_state_controller/command", 50);
 
  // process keyboard input
  std_msgs::Float64 out_msg;
  double speed = 10.0;
  bool stop = false;
  while(ros::ok() && !stop){
    char k = '\0';
    // get symbol from keyboard
    int res = getch(k);
    if(res!=0){
      fprintf(stderr, "Input error, code %i\n", res);
    }else{
      k = tolower(k);
      switch(k){
          case 'w': //go front with specified speed
              out_msg.data = speed;
              l_pub.publish(out_msg);
              r_pub.publish(out_msg);
              break;
              
          case 'a': // go left with specified speed
              out_msg.data = -speed;
              l_pub.publish(out_msg);
              out_msg.data = speed;
              r_pub.publish(out_msg);
              break;
              
          case 'd': // go right with specified speed
              out_msg.data = speed;
              l_pub.publish(out_msg);
              out_msg.data = -speed;
              r_pub.publish(out_msg);
              break;
              
          case 's': // go back with specified speed
              out_msg.data = -speed;
              l_pub.publish(out_msg);
              r_pub.publish(out_msg);
              break;
              
          case 'x': // full stop
              out_msg.data = 0;
              l_pub.publish(out_msg);
              r_pub.publish(out_msg);
              break;
          case 'q':
              stop=true;
      }
    }
  }
  
  return 0;
}
