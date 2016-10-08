#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_emulator():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_emulator')

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        position = Twist()
        #position.linear = [2.0, 2.0, 0.0]
        #position.angular = [0.0, 0.0, 0.3]
        position.linear.x = 1.0
        position.linear.y = 1.0
        position.angular.z = 1.0

        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_emulator()
    except rospy.ROSInterruptException:
        exit()
