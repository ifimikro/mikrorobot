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
mkdir src
cd src
catkin_init_workspace

```

### Fork repoet
Ga til https://github.com/bjornite/mikrorobot, og trykk fork knappen :)

### Clone mikrorobot:
```bash
git clone https://github.com/<ditt brukernavn>/mikrorobot ~/catkin_ws/src/mikrorobot
cd ~/catkin_ws
catkin_make
rospack profile
```
Kjør 
```bash
rospack profile | grep catkin 
``` 
Dersom den produserer tekst, så er alt ok.
Dersom den ikke produserer tekst, sjekk at du har sourca setup.bash
``` bash
source ~/catkin_ws/devel/setup.bash
```

```bash
rosrun mikror<tab>
```
Dette bør autocomplete til mikrorobot

### Last ned ROS Node Automator:
```bash
git clone https://github.com/ymli81/RosNodeAutomator ~/RNA/RosNodeAutomator
```

### Ps:
Det kan være en ide å automatisk source setup.bash til din init fil ( i dette tilfellet .bashrc). eg
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> .bashrc
```
