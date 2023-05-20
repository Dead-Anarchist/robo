#!/usr/bin/env bash
sudo apt-get install ros-$ROS_DISTRO-velodyne* ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-move-base ros-melodic-$ROS_DISTRO-controllers

sudo pip install -U rdflib ply lxml requests pycollada shapely descartes scipy
sudo pip3 install -U ply rdflib networkx matplotlib
