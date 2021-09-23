#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# this node drives and stops robot alternatively every 3.0 seconds
rospy.init_node("red_light_green_light")

# create publisher to announce velocity commands
# drops any message beyond queue size
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)

# create stop message
red_light_twist = Twist()
red_light_twist.linear.x = 0

# create drive message
# positive x is where robot is facing forward
green_light_twist = Twist()
green_light_twist.linear.x = 0.5

# initialize robot state as stopped
driving_forward = False

# stores the change time
light_change_time = rospy.Time.now()

# initializes the sleep rate
rate = rospy.Rate(10)

# continually publish stream of velocity
# mobile base drivers will time out and stop if they don't 
# receive several messages per second
while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
    
    if light_change_time < rospy.Time.now():
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)
    # code could still run, but node will send too many messages
    # could take up an entire CPU core
    rate.sleep()