# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, time, random
import subprocess
import numpy as np
import wave

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty

import alsaaudio
from pydub import AudioSegment
import pygame
from io import BytesIO
from elevenlabs import set_api_key, generate, Voice, VoiceSettings, voices


class ElevenLabs(Node):
    def __init__(self, elevenlabs_api_key, output_device, voice):
        super().__init__('elevenlabs')
        set_api_key(elevenlabs_api_key)
        
        # Define output device
        self.output_device = output_device

        # Define voice properties
        self.voice = voice
        self.voice_id = {v.name:v.voice_id for v in voices()}[voice]
        self.voice_settings = VoiceSettings(stability=0.71, similarity_boost=0.5, style=0.0, use_speaker_boost=False)

        # Define connections
        self.sub = self.create_subscription(String, '/verbaliser/speaker', self.callback, 10)
        self.pub = self.create_publisher(Empty, '/verbaliser/audio_trigger', 10)

    def callback(self, msg):
        print('\n\n')
        self.get_logger().info(msg.data)

        # Idenntify speaker and set format for msg publishing
        sentence_list = msg.data.replace('"',',').replace('   ', '...').replace('...','.').split('...')
        for sentence in sentence_list:
            print('| ', sentence)

            # Generate audio
            audio_bytes = generate(text=sentence, voice=Voice(voice_id=self.voice_id, settings=self.voice_settings))

            # Save audio for playback
            mp3_file_path = 'sentence.mp3'
            with open(mp3_file_path, 'wb') as file:
                file.write(audio_bytes)
            wav_file_path = 'sentence.wav'
            with wave.open(wav_file_path, 'wb') as wav_file:
                # Set the audio parameters
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(44100)
                wav_file.writeframes(audio_bytes)

            # Play Audio file
            self.alsaaudio(audio_bytes)
        
            # Add a pause if more then one sentence
            if sentence is not sentence_list[-1]:
                if random.randint(0, 1):
                    time.sleep(0.25)
                time.sleep(0.75)

        self.pub.publish(Empty())


    def alsaaudio(self, audio):
        # Convert bytearray to an in-memory file-like object
        mp3_io = BytesIO(audio)

        # Load the MP3 file from the in-memory file
        audio_segment = AudioSegment.from_mp3(mp3_io)

        # Resample the audio segment to 100000 Hz
        audio_segment = audio_segment.set_frame_rate(100000)

        # Configure device
        try:
            device = alsaaudio.PCM(device=self.output_device)
            device.setchannels(2)
            device.setrate(100000)
            device.setformat(alsaaudio.PCM_FORMAT_S16_LE)  # Assuming 16-bit samples
            device.setperiodsize(1024)
            
            print('audio_segment.channels', audio_segment.channels)
            print('audio_segment.frame_rate', audio_segment.frame_rate)

            # Prepare audio data for playback
            pcm_data = np.frombuffer(audio_segment.raw_data, dtype=np.int16)

            # Play audio
            try:
                index = 0
                chunk_size = 512  # Since the audio is mono, 512 samples per period
                while index < len(pcm_data):
                    device.write(pcm_data[index:index+chunk_size].tobytes())
                    index += chunk_size
            except alsaaudio.ALSAAudioError:
                print("Data size must be multiple of framesize")

        except alsaaudio.ALSAAudioError:
            print("Device or resource busy ", self.output_device)



def main(args=None):
    rclpy.init(args=args)
    
    elevenlabs_api_key = os.getenv('ELEVENLABS_API_KEY')
    output_device = os.getenv('ALSA_OUTPUT', 'hw:2,0') #aplay -l
    voice = os.getenv('ELEVENLABS_VOICE', 'Sam')

    EL = ElevenLabs(elevenlabs_api_key, output_device, voice)
    #EL.callback(String(data='this is a test'))
    rclpy.spin(EL)

    EL.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
