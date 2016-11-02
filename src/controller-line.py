#!/usr/bin/python
import sys
# Python libs
import sys, time
# numpy and scipy
import numpy as np
# Ros libraries
import rospy
import tf
 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
 
from sys import *
from std_msgs.msg import String
 
## Message
## Topic: sim
## "start" begins motion detection

 
class ctr:
    def __init__(self):
        print "attaching to ROS"
        self.speed = 0.8
        self.last = 0.0
        self.int = 0
        self.last_e = 0
        self.pub = rospy.Publisher('sim_p3at/cmd_vel', Twist, queue_size=10)
#        self.subscriber = rospy.Subscriber("sim_p3at/odom", Odometry, self.callback)
        self.subscriber = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.callback)
        print "wating for data"
 
    def callback(self,data):
        self.loop(data)
 
    def loop(self,data):
        # get dt
        time = data.header.stamp.secs + data.header.stamp.nsecs / 1000000000.0
        dt = time - self.last
        if dt > 10:
            dt = 0
        self.last = time

        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        twist = Twist()

        # speed controller
#        if data.pose.pose.position.x > 5:
#            self.speed = -0.8
#            self.int = 0
#        elif data.pose.pose.position.x < -5:
#            self.speed = 0.8
#            self.int = 0

        #yaw controller
        yaw = euler[2]
        factor = 1;
        if self.speed < 0:
            factor = -1
        
        e = 0 - yaw
        e = 0 - data.pose.pose.position.y
        Kp = 0.3
        Ki = 0.15
        Kd = 0.2

        self.int = self.int + e * dt

        # anti-windup
        if self.int > 0.4 :
            self.int = 0.4
        if self.int < -0.4:
            self.int = -0.4

        dev = (e - self.last_e) / dt
        u = Kp * e + Ki * self.int + Kd * dev

#        if u > 1 :
#            u = 1
#        if u < -1:
#            u = -1
        
        if yaw > 0.5:
            u = min(0,u)
        if yaw < -0.5:
            u = max(0,u)
        self.last_e = e

        rospy.loginfo("position %f,%f yaw %f, dt %f, u %f",data.pose.pose.position.x, data.pose.pose.position.y, euler[2], dt, u)
        twist.linear.x = self.speed
        twist.angular.z = u
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
