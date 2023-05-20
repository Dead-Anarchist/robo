#!/usr/bin/env sh

if [ -z "$1" ]
  then
    BAG_FILE=arctic_data_0
  else
    BAG_FILE=$1
fi

BAG_PATH=~/bags

rosparam set /use_sim_time True

rosrun rosbag play -i ${BAG_PATH}/${BAG_FILE}.bag -d 2 &

rostopic echo /default_robot/encoders -p > ${BAG_PATH}/${BAG_FILE}_encoders.csv &
rostopic echo /default_robot/gazebo_ground_truth_odom -p > ${BAG_PATH}/${BAG_FILE}_odometry.csv &
rostopic echo /default_robot/velodyne_points2 -p > ${BAG_PATH}/${BAG_FILE}_obstacle_cloud.csv &
rostopic echo /default_robot/sonar -p > ${BAG_PATH}/${BAG_FILE}_sonar.csv &
rostopic echo /default_robot/camera1/camera_info -p > ${BAG_PATH}/${BAG_FILE}_camera_info.csv &

