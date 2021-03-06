#!/usr/bin/env python

import threading
import rospy
import actionlib
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from Dixon.msg import command
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Pose


class Nav:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Nav module running ...")

        # init var (for move commands only)
        self.lin_speed = 0.5
        self.lin_dir = 0

        # current global position
        self.current_x = None
        self.current_y = None

        # 0: waiting, 1: moving locally, 2: moving globally
        self.action_flag = 0

        threadObj = threading.Thread(target=self.publishLocal)
        threadObj.start()
        # subscriber
        rospy.Subscriber('Dixon/command', command, self.moveCallback)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.odomCallback)

    def moveCallback(self, msg):
        # check the command
        # this program structure is temp
        # will be changed when nav goals are added
        # will use different functions for different commands
        if msg.local is True:
            if msg.command == "go":
                self.action_flag = 1
		print(msg.destination)
                if msg.destination == "forward":
                    self.lin_dir = 1
                elif msg.destination == "backward":
                    self.lin_dir = -1
        else:
	    print("msg.local not true")
            if msg.command == "go":
		print(msg.command)
		print(msg.x)
		print(msg.y)
                # actionlib send goal
                self.action_flag = 2
                self.moveToGoal(msg.x, msg.y)
        if msg.command == "stop":
            if self.action_flag is 2:
                # actionlib send stop goal
                self.action_flag = 0
                self.moveToGoal(self.current_x, self.current_y)
            elif self.action_flag is 1:
                self.lin_dir = 0
                self.action_flag = 0

        print(msg.command)
        print(self.lin_dir)

        # vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    def odomCallback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def publishLocal(self):
        vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        rate = rospy.Rate(100)    # 100 Hz
	rospy.loginfo("created local thread")
        while not rospy.is_shutdown():
            vel_msg = Twist()

            vel_msg.linear.x = self.lin_speed * self.lin_dir
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
	    if self.action_flag == 1:
	    	rospy.loginfo("publishing local velocity")
            	vel_pub.publish(vel_msg)
            rate.sleep()

    def moveToGoal(self, x, y):
	print("in moveToGoal")
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	
	while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
    		rospy.loginfo("Waiting for move_base action server to respond ...")
    	goal = MoveBaseGoal()

    	# header
    	goal.target_pose.header.frame_id = "map"
    	goal.target_pose.header.stamp = rospy.Time.now()

    	# point and orientation
    	goal.target_pose.pose.position = Point(x, y, 0)
    	goal.target_pose.pose.orientation.x = 0.0
    	goal.target_pose.pose.orientation.y = 0.0
    	goal.target_pose.pose.orientation.z = 0.0
    	goal.target_pose.pose.orientation.w = 1.0

    	rospy.loginfo("Sending Goal ...")
   	ac.send_goal(goal)

    def cleanup(self):
        rospy.loginfo("Shutting down nav module ...")


if __name__ == "__main__":
    rospy.init_node('dixon_nav')
    try:
        Nav()
        rospy.spin()
    except Exception as e:
        print(e)
