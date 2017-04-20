#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String
from Dixon.msg import command
import csv


class Parser:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Parser running ...")

        # init key words
        self.destination = self.genDict('destinations_csv.txt')
        self.move_commands = self.genDict('commands_csv.txt')
        self.aff_resp = self.genList('aff_resp.txt')
        self.neg_resp = self.genList('neg_resp.txt')
        self.names = ["dixon", "dix"]
        self.dest_queue = []

        # dict that contains the full command to be published
        self.full_command = {'command': None, 'destination': None}

        # construct publisher objects
        self.command_msg = command()
        self.resp_msg = String()

        # subscriber(s)
        # /recognizer/output is the topic pocketsphinx pubs to
        rospy.Subscriber('/recognizer/output', String, self.parseCallback)
        # publishers
        self.pub_cmd = rospy.Publisher('/Dixon/command', command, queue_size=10)
        self.resp_pub = rospy.Publisher('dixon_response/response', String, queue_size=10)

    def parseCallback(self, msg):
        rospy.loginfo(msg.data)
        transcript = msg.data.split()

        # check if names in command
        if self.nameCheck(self.names, transcript):
            self.genCommandDict(transcript)
            # generate response
            if self.responseGen(self.full_command, self.aff_resp, self.neg_resp):
                # add command dictionary values to message and publish
                self.command_msg.command = str(self.full_command['command'])
                if self.full_command['command'] == 'stop':
                    self.command_msg.destination = ""
                else:
                    self.command_msg.destination = str(self.full_command['destination'])
                rospy.loginfo(self.full_command)
                rospy.loginfo(str(self.command_msg))
                self.pub_cmd.publish(self.command_msg)
            # add self.resp to message and publish
            self.resp_msg.data = self.resp
            self.resp_pub.publish(self.resp_msg)

    # generate dict of commands as keys with alternative words as lists mapped to their respective commands
    # takes in a csv file
    # the first value in a row is the command, all other values are alternative words
    # (the first command counts as an alternative word. if the same word is included twice an error is thrown)
    def genDict(self, text):
        with open(text, 'r') as csv_file:
            reader = csv.reader(csv_file)
            try:
                dic = dict((key[0], key) for key in reader)
                rospy.loginfo("Generated:\n" + str(dic))
                return dic
            except IndexError:
                rospy.loginfo("IndexError in genList(): Is the same word listed twice?")

    # generate list based on text file
    # splits at newline
    def genList(self, text):
        word_list = []
        with open(text, 'r') as list_text:
            for word in list_text:
                strp_word = word.strip('\n')
                word_list.append(strp_word)
            rospy.loginfo("Generated list: \n" + str(word_list))
            return word_list

    # check if name is in utterance transcription
    def nameCheck(self, names, transcription):
        for name in names:
            if name not in transcription:
                rospy.loginfo("name not detected, invalid command")
                return False
            else:
                return True

    # check each value for every key in the command and destination dictionaries
    # if any of the values matches a word in the transcription add it the full_command dict
    # if more than one destination is given the first one will be added to the command dict,
    # the rest will be added to the queue
    def genCommandDict(self, transcript):
        # clear full command dict and queue
        self.full_command['command'] = None
        self.full_command['destination'] = None
        self.dest_queue.clear()
        for word in transcript:
            for cmd_key, cmd_value in self.move_commands.items():
                if word in cmd_key['stop']:
                    self.full_command['command'] = 'stop'
                    self.full_command['destination'] = ''
                    rospy.loginfo("Added stop to command.")
                elif word in cmd_value:
                    self.full_command['command'] = cmd_value
                    rospy.loginfo("Added " + cmd_value + " to command.")
                else:
                    rospy.loginfo("Command not detected.")
            # if there are no destinations queued add the word to the command dict
            # if there are destinations queued add the word to the queue
            for dest_key, dest_value in self.destination.items():
                if word in dest_value:
                    if not self.dest_queue:
                        self.full_command['destination'] = dest_value
                        rospy.loginfo("Added " + dest_value + " to destination.")
                    elif self.dest_queue:
                        self.dest_queue.append(dest_value)
                        rospy.loginfo("Added " + dest_value + " to dest queue")

    # generate response based on command dictionary
    # if no command is given return neg resp
    # if command other than stop is given but no destination is return neg resp
    # otherwise return pos resp
    def responseGen(self, command, aff, neg):
        if command['command'] is None:
            self.resp = neg[random.randrange(len(neg))]
            rospy.loginfo(self.resp)
            return False
        elif command['destination'] is None and command['command'] is not 'stop':
            self.resp = neg[random.randrange(len(neg))]
            rospy.loginfo(self.resp)
            return False
        # if the command is stop generate "I will stop"
        # if the command is move forward/backward generate "I will move forward/backward"
        # if the command is normal generate "I will move to destination"
        else:
            self.resp = aff[random.randrange(len(aff))] + "\n"
            if command['command'] == 'stop':
                self.resp += "I will " + command['command']
            elif command['destination'] == 'forward' or command['destination'] == 'backward':
                self.resp += "I will" + command['command'] + command['destination']
            else:
                self.resp += "I will " + command['command'] + "to " + command['destination']
            rospy.loginfo(self.resp)
            return True

    def cleanup(self):
        rospy.loginfo("Shutting down parser ...")


if __name__ == "__main__":
    rospy.init_node('dixon_parser')
    try:
        Parser()
        rospy.spin()
    except Exception as e:
        print(e)
