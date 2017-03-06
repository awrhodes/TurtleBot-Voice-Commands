import voice
import pyautogui as pa
import time

speech = voice.Voice()


voice_site = 'http://onlinetonegenerator.com/voice-generator.html'
greeting = 'Hello. My name is Dixon. How can I help you?'

speech.openChrome()
speech.searchSite(voice_site)
speech.selVoice(2553, 678, 2562, 332)
speech.clearField(2624, 489)
speech.Inp(greeting)
