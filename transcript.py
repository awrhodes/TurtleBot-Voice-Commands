import argparse
import sys
from google.cloud import speech

class Transcriber():
    def Transcribe(self):
        transcriber = speech.Client()

        audio = open(sys.argv[1], 'rb') #open audio file
        content = audio.read()

        audio_sample = transcriber.sample(
            content=content,
            source_uri=None,
            encoding='FLAC',
            sample_rate=44100)

        alternatives = transcriber.speech_api.sync_recognize(audio_sample) #transcribe audio

        for alternative in alternatives:
            print('Transcription: {}'.format(alternative.transcript))

Transcriber = Transcriber()
Transcriber.Transcribe()
