#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String
from Dixon.msg import command

class Parser:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # init destination key words from file
        destinations_file = open('destination.txt', 'r')
        self.destination = []
        for word in destinations_file:
            strp_word = word.strip('\n')
            self.destination.append(strp_word)
        destinations_file.close()

        #init command key words from file
        commands_file = open('commands.txt', 'r')
        self.move_commands = []
        for word in commands_file:
            strp_word = word.strip('\n')
            self.move_commands.append(strp_word)
        commands_file.close()

        # init key words
        self.aff_resp = ['Okay', 'Sure thing', 'Will do', 'Roger roger']
        self.neg_resp = ["I didn't catch that", "I didn't understnad"]
        self.name = "dixon"

        self.full_command = {'command':None, 'destination':None}

        rospy.loginfo("Parser running ...")

        # subscriber
        rospy.Subscriber('/recognizer/output', String, self.parseCallback)
        # publisher for nav
        self.pub = rospy.Publisher('/Dixon/command', command, queue_size=10)

    def parseCallback(self, msg):
        rospy.loginfo(msg.data)
        transcript = msg.data.split()

        # clear commmand dict.
        self.full_command['command'] = None
        self.full_command['destination'] = None

        # check if name in command
        if self.nameCheck(self.name, transcript):
            # if command and destination word is in utterance add it to the command dict.
            for word in transcript:
                if word in self.move_commands:
                    if self.full_command['command'] is None:
                        self.full_command['command'] = word
                else:
                    if word in self.destination:
                        self.full_command['destination'] = word

            # generate response
            self.responseGen(self.full_command, self.aff_resp, self.neg_resp)

            # add command dictionary values to message and publish
            command_msg = command()
            command_msg.command = str(self.full_command['command'])
            if self.full_command['command'] == 'stop' or self.full_command['command'] == 'halt':
                command_msg.destination = None
            else:
                command_msg.destination = str(self.full_command['destination'])
            print(self.full_command)
            print(str(command_msg))

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
        elif command['command'] == 'stop' or command['command'] == 'halt':
            print("\n"+aff[random.randrange(len(aff))])
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
    except Exception as e:
        print(e)
