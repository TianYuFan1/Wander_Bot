#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Wander:
    
    def __init__(self):
        # Initialize this node
        rospy.init_node("wander")
        # Subscribers
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # Initialize class variables
        self.range_ahead = 1
        self.state_change_time = rospy.Time.now()
        self.driving_forward = True
        self.rate = rospy.Rate(10)
    
    def scan_callback(self, msg):
        # self.range_ahead = min(msg.ranges)
        self.range_ahead = msg.ranges[0]
        
    def run(self):
        while not rospy.is_shutdown():
            print(self.range_ahead)
            if self.driving_forward:
                if (self.range_ahead < 0.8 or rospy.Time.now() > self.state_change_time):
                    self.driving_forward = False
                    self.state_change_time = rospy.Time.now() + rospy.Duration(5)
            else: 
                if rospy.Time.now() > self.state_change_time:
                    self.driving_forward = True
                    self.state_change_time = rospy.Time.now() + rospy.Duration(30)
            
            twist = Twist()
            if self.driving_forward:
                twist.linear.x = 0.2
            else:
                twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
    
wander = Wander()
wander.run()