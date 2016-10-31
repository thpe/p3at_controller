#!/usr/bin/python
import sys
# Python libs
import sys, time
# numpy and scipy
import numpy as np
# Ros libraries
import rospy
 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
 
from sys import *
from std_msgs.msg import String
 
## Message
## Topic: sim
## "start" begins motion detection

 
class ctr:
    def __init__(self):
        print "attaching to ROS"
        self.pub = rospy.Publisher('sim_p3at/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("sim_p3at/odom", Odometry, self.callback)
        self.speed = 0.4
 
    def callback(self,data):
        self.loop(data)
 
    def loop(self,data):
        rospy.loginfo("position %f,%f",data.pose.pose.position.x, data.pose.pose.position.y)
        twist = Twist()
        if data.pose.pose.position.x > 5:
            self.speed = -0.4
        elif data.pose.pose.position.x < -5:
            self.speed = 0.4

        twist.linear.x = self.speed
        twist.angular.z = 0
        self.pub.publish(twist)
 
 
def main(args):
    print "starting controller"
    ctr()
    rospy.init_node('ctr', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
 
if __name__ == '__main__':
    main(sys.argv)
