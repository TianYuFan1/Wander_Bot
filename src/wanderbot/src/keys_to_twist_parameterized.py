#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# The mapping of key to movement direction
key_mapping = {'w': [0,1], 'a': [1,0], 's': [0,-1], 'd': [-1,0], 'x': [0,0]}

# Define global variables
g_last_twist = None
g_vel_scales = [0.1, 0.1]

# Callback, when key is pressed
def keys_callback(msg, twist_pub):
    # Use global variables
    global g_last_twist, g_vel_scales
    # Check if message is valid
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return 
    # Get velocity
    vels = key_mapping[msg.data[0]]
    # Construct twist message
    g_last_twist.angular.z = vels[0] * g_vel_scales[0]
    g_last_twist.linear.x = vels[1] * g_vel_scales[1]
    # Publish the message
    twist_pub.publish(g_last_twist)

if __name__ == "__main__":
    # Initialize this node
    rospy.init_node("keys_to_twist")
    # Publisher
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    # Subscriber
    rospy.Subscriber("keys", String, keys_callback, twist_pub)
    # Initialize twist
    g_last_twist = Twist()
    # Linear param
    if rospy.has_param("~linear_scale"):
        g_vel_scales[1] = rospy.get_param("~linear_scale")
    else:
        rospy.logwarn("linear scale not provided: using %.1f"%g_vel_scales[1])
    # Angular param
    if rospy.has_param("~angular_scale"):
        g_vel_scales[0] = rospy.get_param("~angular_scale")
    else:
        rospy.logwarn("angular scale not provided: using %.1f"%g_vel_scales[0])
    # Define rate
    rate = rospy.Rate(10)
    # Continuously publish message
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()