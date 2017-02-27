import sounddevice as sd
import numpy as np
import soundfile as sf
import sys

class Recorder:
    'Microphone recording object.' #Documentation

    def callback(indata, frames, time, status):
            volume_norm = np.linalg.norm(indata)*10
            if volume_norm > 85:
                try:
                    recording = sd.rec(int((int(sys.argv[1]) * 44100)),
                                       samplerate=44100,channels=1,
                                       blocking=True)
                    sf.write(sys.argv[2],recording,44100,'PCM_16')
                except IndexError:
                    print("Line 24.")
                    print("Error: Filename to save recording to not specified.")
                except ValueError:
                    print("Line 21.")
                    print("Error: Duration passed not int.")

    with sd.InputStream(callback=callback):
        try:
             sd.sleep(int(sys.argv[1]) * 1000)
        except TypeError:
            print("Line 34.")
            print("Error: Duration passed not int.")
