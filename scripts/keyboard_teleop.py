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
import getch

## message import format:
##from MY_PACKAGE_NAME.msg import MY_MESSAGE_NAME
#from   mikrorobot.msg import  UInt16

from std_msgs.msg import UInt16


##############################################################
##   Message Callbacks


##############################################################
##  Service Callbacks


##############################################################
##  Functions

def getDirection():
    c = getch()
    if c == 'w':
        return 1
    elif c == 's':
        return 4
    elif c == 'a':
        return 2
    elif c == 'd':
        return 3
    elif c == ' ':
        return 0


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()

##############################################################
# Main Program Code
# This is run once when the node is brought up (roslaunch or rosrun)
if __name__ == '__main__':
    print "Hello world"
    # get the node started first so that logging works from the get-go
    rospy.init_node("keyboard_teleop")
    rospy.loginfo("Started template python node: keyboard_teleop.")
    ##############################################################
##  Service Advertisers


##############################################################
##  Message Subscribers


##############################################################
##  Message Publishers
UInt16_pub1  = rospy.Publisher("motor",UInt16, queue_size=1000)
rate = rospy.Rate(10) # 10hz

##############################################################
##  Service Client Inits



############# Message Object for Publisher ####################
UInt16_obj1 = UInt16()


############# Service Object for client ####################


##############################################################
##  Main loop start
while not rospy.is_shutdown():

    UInt16_obj1.data = getDirection()


##############################################################
##  Message Publications

    UInt16_pub1.publish(UInt16_obj1)

##############################################################
##  Service Client Calls


    rospy.loginfo("keyboard_teleop: main loop")
    print(UInt16_obj1.data)
    rate.sleep()
    ###############################################################
#
# end of main wile loop


##############################################################
## Classes

