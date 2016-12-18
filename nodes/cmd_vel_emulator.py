#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

def cmd_vel_emulator():

    pub = rospy.Publisher('/mikrorobot/cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_emulator')

    rate = rospy.Rate(10) # 10hz
    time = rospy.get_time()
    while not rospy.is_shutdown():
        x = 0.3
        y = 0.0
        z = 0.0
        now = rospy.get_time()
        if now - time >= 3.33:
            x = 0.0

        position = Twist()
        position.linear.x = x
        position.linear.y = y
        position.angular.z = z
        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_emulator()
    except rospy.ROSInterruptException:
        exit()
