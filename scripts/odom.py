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
from __future__ import division
import rospy
import roslib; roslib.load_manifest('mikrorobot')


from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

R = 0.1
L1 = 0.198 # bredde
L2 = 0.184 # lengde
L = 1.0/(L1+L2)
translation_matrix = np.matrix([[1.0, 1.0, 1.0, 1.0], [1.0, -1.0, -1.0, 1.0], [-L, L, -L, L]])
translation_matrix *= (R/4.0)

# error constants
error_per_meter = 0.2
error_per_radian = 0.1
lin_vel_error = 0.1
ang_vel_error = 0.1

# sum errors
sum_error_x = 0.0
sum_error_y = 0.0
sum_error_th = 0.0

# covariance matrix for updating
cov = np.matrix('0.0,0.0,0.0,0.0,0.0,0.0;0.0,0.0,0.0,0.0,0.0,0.0;0.0,0.0,0.0,0.0,0.0,0.0;0.0,0.0,0.0,0.0,0.0,0.0;0.0,0.0,0.0,0.0,0.0,0.0;0.0,0.0,0.0,0.0,0.0,0.0')

# times for calculating position
last_time = 0.0
current_time = 0.0
first_publish = 1

# position
x = 0.0
y = 0.0
th = 0.0

def euler_to_quaternion(pitch, roll, yaw):
    t0 = np.cos(yaw/2.0)
    t1 = np.sin(yaw/2.0)
    t2 = np.cos(roll/2.0)
    t3 = np.sin(roll/2.0)
    t4 = np.cos(pitch/2.0)
    t5 = np.sin(pitch/2.0)

    w = t0*t2*t4 + t1*t3*t5
    x = t0*t3*t4 - t1*t2*t5
    y = t0*t2*t5 + t1*t3*t4
    z = t1*t2*t4 - t3*t5*t0
    return [x, y, z, w]


##############################################################
##   Message Callbacks
def motor_speeds_cb(JointState):
  rospy.loginfo("odom: I got message on topic motor_speeds")
  current_time = rospy.get_time()
  global th, x, y, last_time, sum_error_y, sum_error_x, sum_error_th, lin_vel_error, first_publish

  w1 = JointState.velocity[1]
  w2 = JointState.velocity[0]
  w3 = JointState.velocity[3]
  w4 = JointState.velocity[2]

  # flipping the message from motor_speeds_with_names for doing the right calculations
  rpm = [[w1], [w2], [w3], [w4]]
  vel = [[a * ((2*math.pi)/60) for a in b] for b in rpm]
  #vel /= k
  twist = translation_matrix * vel

  # set twist to very small values in the first callback to generate some data
  if first_publish == 1:
      twist = [0.0001, 0.0001, 0.0001]
      first_publish = 0

  delta_time = current_time - last_time

  # meters
  delta_x = twist[0] * delta_time
  delta_y = twist[1] * delta_time
  delta_th = twist[2] * delta_time

  # errors
  error_x = delta_x * error_per_meter
  error_y = delta_y * error_per_meter
  error_th = delta_th * error_per_radian

  # values to put in covariance matrix
  vel_error_x = twist[0] * lin_vel_error
  vel_error_y = twist[1] * lin_vel_error
  vel_error_th = twist[2]  * ang_vel_error
  sum_error_x += error_x
  sum_error_y += error_y
  sum_error_th += error_th

  # position over time
  x += (delta_x * np.cos(delta_th) - delta_y * np.sin(delta_th))
  y += (delta_y * np.cos(delta_th) + delta_x * np.sin(delta_th))
  th += delta_th

  # create quaternion for odom pose orientation
  quat = euler_to_quaternion(0, 0, th)

  Odom_obj1.header.stamp = rospy.Time.now()
  Odom_obj1.header.frame_id = "odom"
  Odom_obj1.child_frame_id = "base_link"
  Odom_obj1.twist.twist.linear.x = twist[0]
  Odom_obj1.twist.twist.linear.y = twist[1]
  Odom_obj1.twist.twist.angular.z = twist[2]
  Odom_obj1.twist.covariance[0] = vel_error_x
  Odom_obj1.twist.covariance[7] = vel_error_y
  Odom_obj1.twist.covariance[14] = 1000000
  Odom_obj1.twist.covariance[21] = 1000000
  Odom_obj1.twist.covariance[28] = 1000000
  Odom_obj1.twist.covariance[35] = vel_error_th
  Odom_obj1.pose.pose.position.x = x
  Odom_obj1.pose.pose.position.y = y
  Odom_obj1.pose.pose.position.z = 0.0
  Odom_obj1.pose.pose.orientation.x = quat[0]
  Odom_obj1.pose.pose.orientation.y = quat[1]
  Odom_obj1.pose.pose.orientation.z = quat[2]
  Odom_obj1.pose.pose.orientation.w = quat[3]
  Odom_obj1.pose.covariance[0] = sum_error_x
  Odom_obj1.pose.covariance[7] = sum_error_y
  Odom_obj1.pose.covariance[14] = 1000000
  Odom_obj1.pose.covariance[21] = 1000000
  Odom_obj1.pose.covariance[28] = 1000000
  Odom_obj1.pose.covariance[35] = sum_error_th
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
  JointState_sub1 = rospy.Subscriber("motor_speed_with_names", JointState, motor_speeds_cb)
  current_time = rospy.get_time()
  last_time = rospy.get_time()


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
