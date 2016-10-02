#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState

class PublisherAndSubscriber:
    js = JointState()

    def _init_(self):
        rospy.init_node('motor_speed_converter')
        print("Node initialized")
        self.pub = rospy.Publisher('motor_speed_with_names', JointState, queue_size=10)
        self.sub = rospy.Subscriber('motor_speeds', JointState, self.fixMessage)
        print("Created subscriber and publisher")
    
    
    def fixMessage(self,motor_cmds):
        self.js.name = ["front_right_wheel_joint", "back_right_wheel_joint", "front_left_wheel_joint", "back_left_wheel_joint"]
        self.js.velocity = motor_cmds.velocity
        self.js.header.stamp = rospy.Time.now()
        self.pub.publish(self.js)

    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    try:
        PASObject = PublisherAndSubscriber()
        PASObject._init_()
        PASObject.run()
    except rospy.ROSInterruptException:
        exit()
