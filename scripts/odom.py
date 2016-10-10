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
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf

R = 0.1
L1 = 0.198 # bredde
L2 = 0.184 # lengde
L = 1.0/(L1+L2)
translation_matrix = np.matrix([[1.0, 1.0, 1.0, 1.0], [1.0, -1.0, -1.0, 1.0], [-L, L, -L, L]])
translation_matrix *= (R/4.0)

# times for calculating position
last_time = 0.0
current_time = 0.0

# position
x = 0.0
y = 0.0
z = 0.0

##############################################################
##   Message Callbacks
def motor_speeds_cb(JointState):
  rospy.loginfo("odom: I got message on topic motor_speeds")
  current_time = rospy.Time.now()
  global z, x, y, last_time

  w1 = JointState.velocity[1]
  w2 = JointState.velocity[0]
  w3 = JointState.velocity[3]
  w4 = JointState.velocity[2]

  # flipping the message from base_controller for doing the right calculations
  rpm = [[w1], [w2], [w3], [w4]]
  vel = [[x * ((2*math.pi)/60) for x in y] for y in rpm]
  #vel /= k
  twist = translation_matrix * vel

  delta_time = current_time - last_time
  # meters
  delta_x = twist[0] * delta_time.secs
  delta_y = twist[1] * delta_time.secs
  delta_z = twist[2] * delta_time.secs * (180/math.pi)

  # position over time
  x += delta_x * np.cos(delta_z)
  y += delta_y * np.sin(delta_z)
  z += delta_z

  # create quaternion for odom pose orientation
  quat = tf.transformations.quaternion_from_euler(0.0, 0.0, z)

  #Odom_obj1.pose = [x, y, 0.0, 0.0, 0.0, z]
  #Odom_obj1.twist = [twist[0], twist[1], 0.0, 0.0, 0.0, twist[2]]
  Odom_obj1.twist.twist.linear.x = twist[0]
  Odom_obj1.twist.twist.linear.y = twist[1]
  Odom_obj1.twist.twist.angular.z = twist[2]
  Odom_obj1.pose.pose.position.x = x
  Odom_obj1.pose.pose.position.y = y
  Odom_obj1.pose.pose.position.z = 0.0
  Odom_obj1.pose.pose.orientation.x = quat[0]
  Odom_obj1.pose.pose.orientation.y = quat[1]
  Odom_obj1.pose.pose.orientation.z = quat[2]
  Odom_obj1.pose.pose.orientation.w = quat[3]
  last_time = current_time
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
  JointState_sub1 = rospy.Subscriber("motor_cmds", JointState, motor_speeds_cb)
  current_time = rospy.Time.now()
  last_time = rospy.Time.now()


##############################################################
##  Message Publishers
  Odom_pub1  = rospy.Publisher("odom", Odometry, queue_size=1000)


##############################################################
##  Service Client Inits



############# Message Object for Publisher ####################
  Odom_obj1 = Odometry()

  #Twist_obj1.linear =
  #Twist_obj1.angular =

############# Service Object for client ####################


##############################################################
##  Main loop start
  while not rospy.is_shutdown():
##############################################################
##  Message Publications
    Odom_pub1.publish(Odom_obj1)

##############################################################
##  Service Client Calls


    rospy.loginfo("odom: main loop")
    rospy.sleep(0.1)
###############################################################
#
# end of main wile loop
