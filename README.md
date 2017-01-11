# Mikrorobot

## Setup

### Installer ros kinetic

http://wiki.ros.org/kinetic/Installation/Ubuntu
**OBS: Hvis du bruker en annen terminal enn standard, for eksempel zsh:**
Da vil ikke
```bash
source /opt/ros/kinetic/setup.bash
```
virke. source den riktige configfilen. For zsh vil kommandoen være
```bash
source /opt/ros/kinetic/setup.zsh
```

Installer pakker vi bruker:
```bash
sudo apt-get install ros-kinetic-rosserial
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-teleop-twist-joy
sudo apt-get install ros-kinetic-robot-pose-ekf
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-ros-controllers
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
git checkout kinetic-dev
cd ~/catkin_ws
catkin_make
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
**OBS: Hvis du bruker en annen terminal enn standard, for eksempel zsh:**
vil ikke dette virke.
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
