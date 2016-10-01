#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32MultiArray

def base_controller_emulator():
    
    pub = rospy.Publisher('motor_cmds', Float32MultiArray, queue_size=10)
    rospy.init_node('base_controller_emulator')
    
    rate = rospy.Rate(10) # 10hz
    x = 0
    
    while not rospy.is_shutdown():

        position = Float32MultiArray()
        position.data = [150*math.sin(x%3),160*math.cos(x%2),190*math.sin(x%1.7),200*math.cos(x%1)]        
        x = x + 0.01
        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        base_controller_emulator()
    except rospy.ROSInterruptException:
        exit()

