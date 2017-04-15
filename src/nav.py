#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from Dixon.msg import command

class Nav:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Nav module running ...")

        # init var (for move commands only)
        self.lin_speed = 0.5
        self.lin_dir = 0

        # subscriber
        rospy.Subscriber('Dixon/command', command, self.moveCallback)
        
    def moveCallback(self, msg):
        # check the command
        # this program structure is temp
        # will be changed when nav goals are added
        # will use different functions for different commands
        if msg.command == "move":
            if msg.destination == "forward":
                self.lin_dir = 1
            else if msg.destination == "backwards":
                self.lin_dir = -1
        else if msg.command == "stop":
            self.lin_dir = 0
        
        vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        vel_msg = Twist()

        vel_msg.linear.x = self.lin_speed * self.lin_dir
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        vel_pub.publish(vel_msg)

    def cleanup(self):
        rospy.loginfo("Shutting down nav module ...")

if __name__=="__main__":
    rospy.init_node('dixon_nav')
    try:
        Nav()
        rospy.spin()
    except:
        pass
