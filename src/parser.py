#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String
from Dixon.msg import command

class Parser:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Parser running ...")

        # init key words
        self.destination = self.genList('destination.txt')
        self.move_commands = self.genList('commands.txt')
        self.aff_resp = self.genList('aff_resp.txt')
        self.neg_resp = self.genList('neg_resp.txt')
        self.name = "dixon"

        # dict that contains the full command to be published
        self.full_command = {'command':None, 'destination':None}

        # subscriber(s)
        rospy.Subscriber('/recognizer/output', String, self.parseCallback)
        # publishers
        self.pub_cmd = rospy.Publisher('/Dixon/command', command, queue_size=10)
        self.resp_pub = rospy.Publisher('dixon_response/response', String, queue_size=10)

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
                        print("Added " + word + " to command.")
                else:
                    if word in self.destination:
                        self.full_command['destination'] = word
                        print("Added " + word + " to destination.")

            # generate response
            if self.responseGen(self.full_command, self.aff_resp, self.neg_resp):
                # add command dictionary values to message and publish
                command_msg = command()
                command_msg.command = str(self.full_command['command'])
                if self.full_command['command'] == 'stop' or self.full_command['command'] == 'halt':
                    command_msg.destination = ""
                else:
                    command_msg.destination = str(self.full_command['destination'])
                print(self.full_command)
                print(str(command_msg))
                self.pub_cmd.publish(command_msg)
                
            # add self.resp to message and publish
            resp_msg = String()
            resp_msg.data = self.resp
            self.resp_pub.publish(resp_msg)

    def genList(self, file):
        word_list = []
        with open(file, 'r') as text:
            for word in text:
                strp_word = word.strip('\n')
                word_list.append(strp_word)
                #for debugging
                print(strp_word)
            return word_list

    def nameCheck(self, name, transcription):
        if name not in transcription:
            print("Name not detected, invalid command")
            return False
        else:
            return True

    def responseGen(self, command, aff, neg):
        if command['command'] == None:
            self.resp = neg[random.randrange(len(neg))]
            print(self.resp)
            return False
        elif command['command'] == 'stop' or command['command'] == 'halt':
            self.resp = "\n"+aff[random.randrange(len(aff))]
            print(self.resp)
            return True
        elif command['destination'] == None:
            self.resp = neg[random.randrange(len(neg))]
            print(self.resp)
            return False
        else:
            self.resp = "\n"+aff[random.randrange(len(aff))]
            self.resp += ". I will " + command['command'] + " to " + command['destination'] + "."
            print(self.resp)
            return True

    def cleanup(self):
        rospy.loginfo("Shutting down parser ...")

if __name__=="__main__":
    rospy.init_node('dixon_parser')

    try:
        Parser()
        rospy.spin()
    except Exception as e:
        print(e)
