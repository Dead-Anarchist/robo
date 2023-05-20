#!/usr/bin/env sh

BAG_PATH=~/bags
BAG_FILE=arctic_data.bag

# Maximum size of a single bag file (the recording will be split into several file after exceeding it)
MAX_SPLIT_SIZE=6144

# Maximum number of splits (bag parts), after exceeding the oldest will be written over
MAX_SPLITS=5

# Records various data, including sensors to a ROS bag file
rosrun rosbag record -a -O ${BAG_PATH}/${BAG_FILE} --split --size=$MAX_SPLIT_SIZE --max-splits=$MAX_SPLITS __name:=rosbag_record

