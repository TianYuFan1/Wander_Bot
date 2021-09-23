#!/usr/bin/env python

import rospy
import sys, select, tty, termios
from std_msgs.msg import String

if __name__ == "__main__":
    # Initialize this node
    rospy.init_node("keyboard_driver")
    # Publishers
    key_pub = rospy.Publisher("keys", String, queue_size = 1)
    # Sleep rate
    rate = rospy.Rate(100)

    # Get readings when key is pressed
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("Publishing keystores. Press Ctrl-C to exit...")

    while not rospy.is_shutdown():
        # Have zero timeout, returns immediately
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    # Set the console back into standard mode
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)