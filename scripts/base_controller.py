#!/usr/bin/env python
"""nav.py, A minimal ROS node in Python.
--------------------------------
(C) 2014, Biorobotics Lab, Department of Electrical Engineering, University of Washington
This file is part of RNA - The Ros Node Automator.

    RNA - The Ros Node Automator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RNA - The Ros Node Automator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with RNA - The Ros Node Automator.  If not, see <http://www.gnu.org/licenses/>.
--------------------------------
"""
### This file is generated using ros node template, feel free to edit
### Please finish the TODO part
#  Ros imports
import rospy
import roslib; roslib.load_manifest('mikrorobot')

## message import format:
##from MY_PACKAGE_NAME.msg import MY_MESSAGE_NAME
#from   mikrorobot.msg import  JointState

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np


# avstander fra robotens senter
L1 = 1.0
L2 = 1.0
L = L1+L2
k = 1.0 # utvekslingskonstant
R = 2.0 # hjulradius
translation_matrix = np.matrix([[1.0, 1.0, -L], [1.0, -1.0, L], [1.0, -1.0, -L], [1.0, 1.0, L]])
translation_matrix *= 1.0/R


##############################################################
##   Message Callbacks
def cmd_vel_cb(Twist):
  rospy.loginfo("base_controller: I got message on topic cmd_vel")
  rospy.loginfo("base_controller: cmd_vel Message contains: linear = %", Twist.linear)
  rospy.loginfo("base_controller: cmd_vel Message contains: angular = %", Twist.angular)
  target = np.matrix([[Twist.linear.x], [Twist.linear.y], [Twist.angular.z]])

  wheel_vel = translation_matrix * target
  enginge_vel = wheel_vel / k

  # message to publish
  JointState_obj1.velocity = enginge_vel

##############################################################
##  Service Callbacks


##############################################################
# Main Program Code
# This is run once when the node is brought up (roslaunch or rosrun)
if __name__ == '__main__':
  print "Hello world"
# get the node started first so that logging works from the get-go
  rospy.init_node("base_controller")
  rospy.loginfo("Started template python node: base_controller.")
##############################################################
##  Service Advertisers


##############################################################
##  Message Subscribers
  Twist_sub1 = rospy.Subscriber("cmd_vel", Twist, cmd_vel_cb)


##############################################################
##  Message Publishers
  JointState_pub1  = rospy.Publisher("motor_cmds",JointState, queue_size=1000)


##############################################################
##  Service Client Inits



############# Message Object for Publisher ####################
  JointState_obj1 = JointState()

  JointState_obj1.name = "Motor velocity commands"
  JointState_obj1.position = 0
  JointState_obj1.velocity = 0
  JointState_obj1.effort = 0

############# Service Object for client ####################


##############################################################
##  Main loop start
  while not rospy.is_shutdown():
##############################################################
##  Message Publications
    JointState_pub1.publish(JointState_obj1)

##############################################################
##  Service Client Calls


    rospy.loginfo("base_controller: main loop")
    rospy.sleep(2)
###############################################################
#
# end of main wile loop
