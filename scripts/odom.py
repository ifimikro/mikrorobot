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
#from   mikrorobot.msg import  Twist

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np

R = 1.0
L1 = 1.0
L2 = 1.0
L = 1.0/(L1+L2)
translation_matrix = np.matrix([[1.0, 1.0, 1.0, 1.0], [1.0, -1.0, -1.0, 1.0], [-L, L, -L, L]])
translation_matrix *= (R/4.0)


##############################################################
##   Message Callbacks
def motor_speeds_cb(JointState):
  rospy.loginfo("odom: I got message on topic motor_speeds")
  rospy.loginfo("odom: motor_speeds Message contains: name = %s", JointState.name)
  rospy.loginfo("odom: motor_speeds Message contains: position = %f", JointState.position)
  rospy.loginfo("odom: motor_speeds Message contains: velocity = %f", JointState.velocity)
  rospy.loginfo("odom: motor_speeds Message contains: effort = %f", JointState.effort)

  twist = translation_matrix * np.transpose(JointState.velocity)
  Twist_obj1.linear.x = twist(0)
  Twist_obj1.linear.y = twist(1)
  Twist_obj1.angular.z = twist(2)
##############################################################
##  Service Callbacks


##############################################################
# Main Program Code
# This is run once when the node is brought up (roslaunch or rosrun)
if __name__ == '__main__':
  print "Hello world"
# get the node started first so that logging works from the get-go
  rospy.init_node("odom")
  rospy.loginfo("Started template python node: odom.")
##############################################################
##  Service Advertisers


##############################################################
##  Message Subscribers
  JointState_sub1 = rospy.Subscriber("motor_speeds", JointState, motor_speeds_cb)


##############################################################
##  Message Publishers
  Twist_pub1  = rospy.Publisher("odom",Twist, queue_size=1000)


##############################################################
##  Service Client Inits



############# Message Object for Publisher ####################
  Twist_obj1 = Twist()

  Twist_obj1.linear = 0
  Twist_obj1.angular = 0

############# Service Object for client ####################


##############################################################
##  Main loop start
  while not rospy.is_shutdown():
##############################################################
##  Message Publications
    Twist_pub1.publish(Twist_obj1)

##############################################################
##  Service Client Calls


    rospy.loginfo("odom: main loop")
    rospy.sleep(2)
###############################################################
#
# end of main wile loop
