#!/bin/bash

# Jade installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-jade-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall


# Exstras
sudo apt-get install ros-jade-usb-cam
sudo apt-get install ros-jade-rosserial

mkdir ~/catkin_ws && cd catkin_ws
catkin_init_workspace

git clone https://github.com/bjornite/mikrorobot ~/catkin_ws/src/mikrorobot

cd ~/catkin_ws
catkin_make
rospack profile

echo "Sjekk at utskriften inneholder ~/catkin_ws/src"
git clone https://github.com/ymli81/RosNodeAutomator ~/RNA/RosNodeAutomator

