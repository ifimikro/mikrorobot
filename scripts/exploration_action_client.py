#!/usr/bin/env python
from __future__ import division
import rospy
import roslib
roslib.load_manifest('mikrorobot')

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import actionlib
import numpy as np
import tf

global client, listener

def exploration_cb(OccupancyGrid):
    # calculate an unexplored position for the robot to explore
    # get the current position and pose
    current_map = OccupancyGrid.data
    current_pos = listener.lookupTransform('base_link', 'map', rospy.Time(0))
    print current_pos

    goal = MoveBaseGoal()
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(20.0))

if __name__ == '__main__':
  rospy.init_node("exploration")
  rospy.loginfo("Started template python node: exploration.")

  client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  listener = tf.TransformListener()


  while not rospy.is_shutdown():
      rospy.loginfo("main loop")

      client.wait_for_server()

      rospy.Subscriber("map", OccupancyGrid, exploration_cb)

      rospy.sleep(0.1)
