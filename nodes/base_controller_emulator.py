#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt32MultiArray

def base_controller_emulator():
    
    pub = rospy.Publisher('motor_cmds', UInt32MultiArray, queue_size=10)
    rospy.init_node('base_controller_emulator')
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():

        position = UInt32MultiArray()
        position.data = [0,10,20,30]        

        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        base_controller_emulator()
    except rospy.ROSInterruptException:
        exit()

