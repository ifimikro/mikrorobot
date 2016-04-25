# Mikrorobot

## Setup

### Installer ros jade

http://wiki.ros.org/jade/Installation/Ubuntu

Installer pakker vi bruker:
```bash
sudo apt-get install ros-jade-usb-cam
sudo apt-get install ros-jade-rosserial
```

### Sett opp et directory for ROS:

```bash
cd ~
mkdir catkin_ws
cd catkin_ws
catkin_init_workspace
```

### Clone mikrorobot:
```bash
git clone https://github.com/bjornite/mikrorobot ~/catkin_ws/src/mikrorobot
cd ~/catkin_ws
catkin_make
rospack profile
```
Sjekk at utskriften inneholder ~/catkin_ws/src
```bash
rosrun mikrobo<tab>
```
Dette bør autocomplete til mikrorobot

### Last ned ROS Node Automator:
```bash
git clone https://github.com/ymli81/RosNodeAutomator ~/RNA/RosNodeAutomator
```
