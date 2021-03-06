#!/usr/bin/env python

#import roslib
import rospy
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

class DixonVoice:
    def __init__(self):
        rospy.on_shutdown(self.cleanUp)

        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.wavepath = rospy.get_param("~wavepath", "")

        # Create the sound client object
        self.soundhandle = SoundClient()

        rospy.sleep(1)
        self.soundhandle.stopAll()

        # Announce that we are ready for input
        self.soundhandle.say("Hello. I am Dixon. How can I help you?", self.voice)

        rospy.loginfo("Ready to take commands...")

        # Subscribe to response generator
        rospy.Subscriber('/dixon_response/response', String, self.dixonVoice)

    def dixonVoice(self, msg):
        # Speak
        self.soundhandle.say(msg.data, self.voice)

    def cleanUp(self):
        rospy.loginfo("Shutting down dixon_voice node...")

if __name__ == "__main__":
    rospy.init_node('dixon_voice')
    try:
        DixonVoice()
        rospy.spin()
    except Exception as e:
        print(e)
