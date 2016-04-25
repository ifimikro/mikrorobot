import rospy
from std_msgs.msg import UInt16

#translates from keyboard input to an int the arduino understands

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




def talker():
    pub = rospy.Publisher('motor', UInt16, queue_size=10)
    rospy.init_node('keyboard_motor_control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        direction = getDirection()
        rospy.loginfo(direction)
        pub.publish(direction)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

