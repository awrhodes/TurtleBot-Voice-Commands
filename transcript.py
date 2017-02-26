import argparse
import sys
from google.cloud import speech

def Transcribe(sound):
    transcriber = speech.Client()

    audio = open(sound, 'rb')

    content = audio.read()

    audio_sample = transcriber.sample(
        content=content,
        source_uri=None,
        encoding='FLAC',
        sample_rate=44100)

    alternatives = transcriber.speech_api.sync_recognize(audio_sample)

    for alternative in alternatives:
        print('Transcription: {}'.format(alternative.transcript))

Transcribe(sys.argv[1])
