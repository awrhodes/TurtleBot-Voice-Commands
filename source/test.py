import voice
import pyautogui as pa
import time
import os

speech = voice.Voice()


voice_site = 'http://onlinetonegenerator.com/voice-generator.html'
greeting = 'Hello. My name is Dixon. How can I help you?'

os.system("google-chrome")
#speech.openChrome()
speech.searchSite(voice_site)
speech.selVoice(957, 677, 1023, 322)
time.sleep(1)
speech.clearField(958, 515)
speech.Inp(greeting)
