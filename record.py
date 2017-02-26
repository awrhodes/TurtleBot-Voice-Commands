import sounddevice as sd
import numpy as np
import soundfile as sf
#sd.default.samplerate = 44100

duration = 5

def callback(indata, frames, time, status):
	volume_norm = np.linalg.norm(indata)*10
	if volume_norm > 10:
		recording = sd.rec(int(duration * 44100), samplerate=44100, channels=1, blocking=True)
		sf.write('test.flac', recording, 44100, 'PCM_16')


with sd.InputStream(callback=callback):
	sd.sleep(duration * 1000)
