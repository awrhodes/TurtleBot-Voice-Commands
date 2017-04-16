#!/user/bin/env python

import roslib
import rospy
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

class TalkBack:
    def __init__(self):
        rospy.on_shutdown(self.cleanUp)

        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.wavepath = rospy.get_param("~wavepath", "")

        # Create the sound client object
        self.soundhandle = SoundClient()

        rospy.sleep(1)
        self.soundhandle.stopAll()

        # Announce that we are ready for input
        self.soundhandle.say("Hello. I am Dixon. How can I help you?", self.voice)

        rospy.loginfo("Ready to take commands...")

        # Subscribe to response generator
        rospy.Subscriber('/recognizer/output', String, self.dixonVoice)

    def dixonVoice(self, msg):
        # Speak
        # Currently only repeats what sphinx transcribes
        self.soundhandle.say(msg.data, self.voice)

    def cleanUp(self):
        rospy.loginfo("Shutting down dixonVoice node...")

if __name__ == "__main__":
    rospy.init_node('dixonVoice')
    try:
        dixonVoice()
        rospy.spin()
    except:
        pass
