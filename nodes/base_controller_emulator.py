#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState


def base_controller_emulator():

    pub = rospy.Publisher('motor_cmds', JointState, queue_size=10)
    rospy.init_node('base_controller_emulator')

    rate = rospy.Rate(10)  # 10hz
    x = 0

    while not rospy.is_shutdown():

        position = JointState()
        # position.data = [150*math.sin(x%3),160*math.cos(x%2),190*math.sin(x%1.7),200*math.cos(x%1)]
        position.velocity = [0, 0, 0, 0]
        x = x + 0.01
        # if (position.data[0] > 128):
        #   position.data[0] = 255
        # else:
        #    position.data[0] = 0

        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()


if __name__ == '__main__':
    try:
        base_controller_emulator()
    except rospy.ROSInterruptException:
        exit()
