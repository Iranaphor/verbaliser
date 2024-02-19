import numpy as np

import alsaaudio
from pydub import AudioSegment
from io import BytesIO



# Open the file in binary read mode
mp3_file_path = 'sentence.mp3'
with open(mp3_file_path, 'rb') as file:
    audio_bytes = file.read()

# Convert bytearray to an in-memory file-like object
mp3_io = BytesIO(audio_bytes)

# Load the MP3 file from the in-memory file
audio_segment = AudioSegment.from_mp3(mp3_io)

# Resample the audio segment to 48000 Hz
audio_segment = audio_segment.set_frame_rate(100000)

# Configure device
output_device = 'hw:2,0'
device = alsaaudio.PCM(device=output_device)
device.setchannels(1)
device.setrate(100000)
device.setformat(alsaaudio.PCM_FORMAT_S16_LE)  # Assuming 16-bit samples
device.setperiodsize(1024)

print('audio_segment.channels', audio_segment.channels)
print('audio_segment.frame_rate', audio_segment.frame_rate)

# Prepare audio data for playback
pcm_data = np.frombuffer(audio_segment.raw_data, dtype=np.int16)


# Play audio
index = 0
chunk_size = 512  # Since the audio is mono, 512 samples per period
while index < len(pcm_data):
    device.write(pcm_data[index:index+chunk_size].tobytes())
    index += chunk_size