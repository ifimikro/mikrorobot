#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16


def commandline_servo_control():

    pub = rospy.Publisher('servo_position', UInt16, queue_size=10)
    rospy.init_node('commandline_servo_control')

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        position = 0
        tmp = input()

        if tmp != 0:
            position = tmp

        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()


if __name__ == '__main__':
    try:
        commandline_servo_control()
    except rospy.ROSInterruptException:
        pass
