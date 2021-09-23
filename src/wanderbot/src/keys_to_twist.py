#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# The mapping of key to movement direction
key_mapping = {'w': [0,1], 'a': [1,0], 's': [0,-1], 'd': [-1,0], 'x': [0,0]}

# Saves the last twist message
g_last_twist = None

# Callback, triggered when key is pressed
def keys_callback(msg, twist_pub):
    global g_last_twist
    # If invalid key message received, return 
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return
    # Get velocity command from key
    vels = key_mapping[msg.data[0]]
    # Create twist message based on map
    g_last_twist.angular.z = vels[0]
    g_last_twist.linear.x = vels[1]
    # Publish twist message
    twist_pub.publish(g_last_twist)

if __name__ == '__main__':
    # Initialize this node
    rospy.init_node("keys_to_twist")
    # Publishers
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    # Subscribers
    rospy.Subscriber("keys", String, keys_callback, twist_pub)
    # Define rate
    rate = rospy.Rate(10)
    # Define twist message
    g_last_twist = Twist()
    # Keep on publishing messages
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()

