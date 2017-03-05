import sounddevice as sd
import numpy as np
import soundfile as sf
import sys

#default values for arguments
duration = 5
filename = "command.flac"
threshold = 85
debug = False

#if sys.argv[1] != None:
#    duration = sys.argv[1]

#if sys.argv[2] != None:
#    filename = sys.argv[2]

#if sys.argv[3] != None:
#    threshold = sys.argv[3]

#if sys.argv[4] == "debug":
#    debug = True

#print("Running with:\nduration:\t" + duration + 
#		"\nfilename:\t" + filename + "\nthreshold:\t" + threshold)

def callback(indata, frames, time, status):
    volume_norm = np.linalg.norm(indata)*10
    if debug == True:
        print("[debug] volume_norm: " + str(volume_norm))

    if volume_norm > threshold:
        try:
            recording = sd.rec(int(duration) * 44100,samplerate=44100,channels=1,blocking=True)
            sf.write(filename,recording,44100,'PCM_16')
        except IndexError:
            print("Line 24.")
            print("Error: Filename to save recording to not specified.")
        except ValueError:
            print("Line 21.")
            print("Error: Duration passed not int.")

with sd.InputStream(callback=callback):
    try:
        sd.sleep(int(duration) * 1000)
    except TypeError:
        print("Line 34.")
        print("Error: Duration passed not int.")
