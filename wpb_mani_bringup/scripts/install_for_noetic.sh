#!/bin/bash
sudo apt-get install -y ros-noetic-joint-state-publisher-gui
sudo apt-get install -y ros-noetic-joy
sudo apt-get install -y ros-noetic-hector-mapping
sudo apt-get install -y ros-noetic-gmapping
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-cv-bridge
sudo apt-get install -y ros-noetic-controller-manager
sudo apt-get install -y ros-noetic-moveit
cd ~/catkin_ws/src
git clone https://gitee.com/s-robot/ar_track_alvar.git