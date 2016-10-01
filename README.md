# Mikrorobot

## Setup

### Installer ros jade

http://wiki.ros.org/jade/Installation/Ubuntu
* OBS: Hvis du bruker en annen terminal enn standard, for eksempel zsh: *
Da vil ikke
```bash
source /opt/ros/<ditro>/setup.bash
```
virke. source den riktige configfilen. For zsh vil kommandoen være
```bash
source /opt/ros/<ditro>/setup.zsh
```

Etter at ROS er installert, installer robotsimulatoren Gazebo.

```bash
sudo apt-get install gazebo5
sudo apt-get install -y libgazebo5-dev
```
Sjekk at Gazebo virker ved å kjøre det fra terminalen.

Installer pakker vi bruker:
```bash
sudo apt-get install ros-jade-usb-cam
sudo apt-get install ros-jade-rosserial
sudo apt-get install ros-jade-navigation
sudo apt-get install ros-jade-joy
sudo apt-get install ros-jade-teleop-twist-joy
sudo apt-get install ros-jade-robot-pose-ekf
sudo apt-get install ros-jade-gmapping
sudo apt-get install ros-jade-amcl
sudo apt-get install ros-jade-gazebo-ros-pkgs
sudo apt-get install ros-jade-ros-control
sudo apt-get install ros-jade-ros-controllers
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
Gå til https://github.com/ifimikro/mikrorobot, og trykk på fork knappen :)

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
* OBS: Hvis du bruker en annen terminal enn standard, for eksempel zsh: *
Da vil ikke dette virke.
source den riktige configfilen. For zsh vil kommandoen være
```bash
source ~/catkin_ws/devel/setup.zsh
```

```bash
rosrun mikror<tab>
```
Dette bør autocomplete til mikrorobot

### Last ned ROS Node Automator:
```bash
git clone https://github.com/ymli81/RosNodeAutomator ~/RNA/RosNodeAutomator
```

##### Ps:
Det kan være en ide å automatisk source setup.bash til din init fil ( i dette tilfellet .bashrc). eg
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> .bashrc
```
