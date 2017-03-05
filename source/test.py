from __future__ import print_function
import speech_recognition as sr
from pocketsphinx.pocketsphinx import *

recognizer = sr.Recognizer() #creates new recognizer object

#dictionary = [("fenn hall", 0.4), ("testing", 0.6)]

#threshold volume to start recording
recognizer.energy_threshold = 9050
print("Set volume threshold to: " + str(recognizer.energy_threshold))

#one second until phrase stops being recorded
recognizer.pause_threshold = 1
print("Set pause threshold to: " + str(recognizer.pause_threshold))

recognizer.operation_timeout = 5 #sets API timeout to 5 sec

fenn = [("Fenn Hall", 0.6)]

with sr.Microphone() as source:
#    recognizer.adjust_for_ambient_noise(source)
    try:
        audio = recognizer.listen(source, timeout=5)
        transcript = recognizer.recognize_sphinx(audio,language='en-US',
                                                 keyword_entries=None,
                                                 show_all=False)
        #for best, i in zip(transcript.nbest(), range(100)):
        #    print(best.hypstr, best.score)
        print(transcript)
    except sr.WaitTimeoutError:
        print("Timed out")
    except sr.UnknownValueError:
        print("Unintelligible speech")
    except sr.RequestError:
        print("RequestError: Something is wrong with Sphinx")
