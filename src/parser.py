#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String
from Dixon.msg import command

class Parser:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # init words
        self.move_commands = ['move', 'go', 'head']
        self.full_command = {'command':None, 'destination':None}
        self.aff_resp = ['Okay', 'Sure thing', 'Will do', 'Roger roger']
        self.neg_resp = ["I didn't catch that", "I didn't understnad"]
        self.destination = ['room 202', 'forward', 'backwards']

        self.name = "dixon"

        rospy.loginfo("Parser running ...")

        # subscriber
        rospy.Subscriber('/recognizer/output', String, self.parseCallback)
        
        # publisher for nav
        self.pub = rospy.Publisher('/Dixon/command', command, queue_size=10)

    def parseCallback(self, msg):
        rospy.loginfo(msg.data)
        transcript = msg.data.split()
        
        # check if name in command
        if self.nameCheck(self.name, transcript):
            for word in transcript:
                # if current word is a known command
                if word in self.move_commands:
                    if self.full_command['command'] is None:
                        self.full_command['command'] = word
                else:
                    if word in self.destination:
                        self.full_command['destination'] = word
            self.responseGen(self.full_command, self.aff_resp, self.neg_resp)
            command_msg = command()
            #command_msg.command = self.full_command['command']
            #command_msg.destination = self.full_command['destination']
            command_msg.command = "apple"
            command_msg.destination = "orange"
            
            self.pub.publish(command_msg)

    def nameCheck(self, name, transcription):
        if name not in transcription:
            print("Name not detected, invalid command")
            return False
        else:
            return True
        
    def responseGen(self, command, aff, neg):
        if command['command'] == None:
            print(neg[random.randrange(len(neg))])
        else:
            print("\n"+aff[random.randrange(len(aff))])
            print("I will " + command['command'] + " to " + command['destination'] + ".")

    def cleanup(self):
        rospy.loginfo("Shutting down parser ...")

if __name__=="__main__":
    rospy.init_node('dixon_parser')
    try:
        Parser()
        rospy.spin()
    except:
        pass
