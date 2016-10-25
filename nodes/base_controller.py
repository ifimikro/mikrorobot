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
import tf
import numpy as np
import math

# 16/24 tenner
# avstander fra robotens sentrum
# bredde og lengde stemmer ikke helt
L1 = 0.198 # bredde
L2 = 0.184 # lengde
L = L1+L2
R = 0.1 # hjulradius
translation_matrix = np.matrix([[1.0, 1.0, -L], [1.0, -1.0, L], [1.0, -1.0, -L], [1.0, 1.0, L]])
translation_matrix *= 1.0/R

def limit_speeds(x):
    new = 0
    if x > 0 and x > 0.3:
        new = 0.3
    if x < 0 and x < -0.3:
        new = -0.3
    else:
        new = x
    return new

def limit_rpm(wheels):
    max_rpms = [max(wheels), min(wheels)]

    limit_factor = 0.0
    # limit the wheel rpms based on the highest one
    if (max_rpms[0] < abs(max_rpms[1]) and max_rpms[1] < -119):
        limit_factor = (max_rpms[1] + 119) / max_rpms[1]
    elif (max_rpms[0] > 119):
        limit_factor = (max_rpms[0] - 119) / max_rpms[0]

    return np.multiply(wheels, limit_factor)



##############################################################
##   Message Callbacks
def cmd_vel_cb(Twist):
  rospy.loginfo("base_controller: I got message on topic cmd_vel")
  #rospy.loginfo("base_controller: cmd_vel Message contains: linear = %d", Twist.linear)
  #rospy.loginfo("base_controller: cmd_vel Message contains: angular = %d", Twist.angular)

  #tf_listener.lookupTransform("front_right_wheel_joint", "base_link", rospy.get_time())
  x = Twist.linear.x
  y = Twist.linear.y
  z = Twist.angular.z

  target = np.matrix([[x], [y], [z]])

  wheel_vel = translation_matrix * target # rad/s

  #convert to rpm
  wheel_vel *= (60.0/(2*math.pi))
  wheel_vel = limit_rpm(wheel_vel)
  # message to publish
  # flipping message to correspond to the motor script
  message = [wheel_vel[1], wheel_vel[0], wheel_vel[3], wheel_vel[2]]
  JointState_obj1.velocity = message

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
  #tf_listener = tf.TransformListener()
  #print tf_listener.lookupTransform("front_right_wheel_joint", "base_link", rospy.Time.now())
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

  #JointState_obj1.name = "Motor velocity commands"
  JointState_obj1.position = []
  JointState_obj1.velocity = []
  JointState_obj1.effort = []

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
    rospy.sleep(0.1)
###############################################################
#
# end of main wile loop
