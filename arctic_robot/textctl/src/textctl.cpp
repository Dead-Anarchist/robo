#include<cstdio>
#include<boost/bind.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

  bool ans_received = false;

void textansCallback(const std_msgs::String::ConstPtr& msg) {
    // print answer
  printf("%s\n", msg->data.c_str());
  ans_received = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "textctl");
  ros::NodeHandle n;
  // create topics to interact with server
  ros::Publisher cmd_pub = n.advertise<std_msgs::String>("default_robot/textcmd", 10);
  ros::Subscriber sc = n.subscribe<std_msgs::String>("default_robot/textans", 10, textansCallback);
                                //boost::bind(textansCallback,
                                //         _1,
                                //         boost::ref(ans_received)));
  // chat with user
  std_msgs::String out_msg;
  ros::Rate r(10);
  while(ros::ok()){
      char cmd[80];
      // read command
      char* r_num = fgets(cmd, 80, stdin);
      if (r_num == NULL ){
	  fprintf(stderr, "Reading error\n");
	  return -1;
      }
      // remove new_string symbol
      int len = strlen(cmd);
      if (cmd[len-1] == '\n')
	  cmd[len-1] = '\0';
      // quit if necesssary
      if (strcmp(cmd, "quit") == 0)
          break;
      // publish it
      out_msg.data = std::string(cmd);
      cmd_pub.publish(out_msg);
      // wait for answer
      while(!ans_received && ros::ok()){
          ros::spinOnce();
          r.sleep();
      }
      ans_received = false;
  }
  return 0;
}
