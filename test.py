from __future__ import print_function
import speech_recognition as sr

#Select microphone
m = None #selected mic
for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
    if microphone_name == "Logitech G430 Gaming Headset: USB Audio (hw:2,0)":
        m = sr.Microphone(device_index=i)
        print("Selected " + microphone_name  + " mic.")
        break
    elif microphone_name == "HDA Intel HDMI: 0 (hw:0,3)":
        m = sr.Microphone(device_index=i)
        print()
        print("Selected " + microphone_name + " mic.")

recognizer = sr.Recognizer() #creates new recognizer object

#threshold volume to start recording
recognizer.energy_threshold = 150
print("Set volume threshold to: " + str(recognizer.energy_threshold))

#one second until phrase stops being recorded
recognizer.pause_threshold = 1
print("Set pause threshold to: " + str(recognizer.pause_threshold))

recognizer.operation_timeout = 5 #sets API timeout to 5 sec

with sr.Microphone() as source:
    audio = recognizer.listen(source, timeout=5)
    transcript = recognizer.recognize_google(audio,key=None,
                                             language='en-US',
                                             show_all=False)
    print(transcript)
