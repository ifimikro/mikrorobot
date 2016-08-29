The files in this folder describe the robot model used by the gazebo simulator, and the robot model used by ROS.


URDF stand for Universal Robot Description Format, see http://wiki.ros.org/urdf



xacro is a macro language for generating urdf-files. If you want to change the robot model, change the xacro file and then run

rosrun xacro xacro --inorder  mikrorobot.xacro > mikrorobot.urdf

to generate the urdf file.



the .gazebo file contains plugins used by gazebo to talk to ROS.

Use this command to spawn the model in gazebo: rosrun gazebo_ros spawn_model -file `rospack find mikrorobot`/urdf/mikrorobot.urdf -urdf -z 1 -model mikrorobot
