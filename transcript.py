import argparse
import sys
from google.cloud import speech

class Transcriber():
    "Transcriber object. Transcribe(audio) to transcribe." #Documentation

    def __init__(self):
        self.transcription = []
        self.audio = sys.argv[1]

    #open audio and transcribe it
    def Transcribe(self, sound):
        transcriber = speech.Client() #instantiate API client

        audio = open(sound, 'rb')
        content = audio.read()

        audio_sample = transcriber.sample(
            content=content,
            source_uri=None,
            encoding='FLAC',
            sample_rate=44100)

        #transcribe audio
        alternatives = transcriber.speech_api.sync_recognize(audio_sample)

        #split the transcription at whitespace, then add the words to a list
        for alternative in alternatives:
            print('Transcription: {}'.format(alternative.transcript))
            self.transcription = alternative.transcript.split(' ')

        return self.transcription
