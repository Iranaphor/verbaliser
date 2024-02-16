#!/usr/bin/env python3

# @desc: note, this example requires PyAudio because it uses the Microphone class
# @url: https://github.com/Uberi/speech_recognition/blob/master/examples/microphone_recognition.py

import os, sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

import speech_recognition as sr


class AudioCollector(Node):

    def __init__(self, in_device, out_device):
        super().__init__('audio_collector')
        self.mic_test_sub = self.create_subscription(String, '/verbaliser/mic_test', self.tests_sub, 10)

        # Get specified microphone device
        self.out_device = out_device
        self.device = in_device
        self.device_list = []
        print('\n\n\n')
        print('identifying mic')
        while True not in self.device_list:
            mic_list = sr.Microphone.list_microphone_names()
            self.device_list = [in_device in m for m in mic_list]
            if True not in self.device_list:
                print('\n\n\n')
                print(f"Device {in_device} not found. Will check again in 5 seconds.")
                [print(m) for m in mic_list]
                print('\n\n')
            time.sleep(5)
        self.device_id = self.device_list.index(True)

        print('\n\n\n')
        print('mic found')
        self.device_name = sr.Microphone.list_microphone_names()[self.device_id]
        self.chunk_size = 1024 if 'K66' not in self.device_name else 512
        print(f'Using device {self.device_id}, named {self.device_name}')

        # Construct recorder
        self.say_failure = False
        self.recogniser = sr.Recognizer()
        self.say = self.create_publisher(String, '/verbaliser/speaker', 10)

        # Construct ROS connections
        self.pub1 = self.create_publisher(String, '/verbaliser/smart_ai_input', 10)
        self.pub2 = self.create_publisher(String, '/verbaliser/basic_ai_input', 10)
        self.pub3 = self.create_publisher(String, '/verbaliser/function_ai_input', 10)

        self.pub0 = self.create_publisher(Empty, '/verbaliser/audio_trigger', 10)
        self.sub0 = self.create_subscription(Empty, '/verbaliser/audio_trigger', self.trigger1, 10)
        self.sub1 = self.create_subscription(Empty, '/verbaliser/smart_ai_audio_trigger', self.trigger1, 10)
        self.sub2 = self.create_subscription(Empty, '/verbaliser/basic_ai_audio_trigger', self.trigger2, 10)
        self.sub3 = self.create_subscription(Empty, '/verbaliser/function_ai_audio_trigger', self.trigger3, 10)

        self.trigger1(Empty())



    def tests_sub(self, msg):

        try:
            # Collect list of microphones
            print(f'searching for {msg.data}')
            mic_list = sr.Microphone.list_microphone_names()
            from pprint import pprint
            pprint(mic_list)
            id, rate, chunk = msg.data.split('~')
            device_list = [id in m for m in mic_list]
            device_id = device_list.index(True)
            device_name = mic_list[device_id]
            recogniser = sr.Recognizer()

            print('\n'*10)
            print(device_list)
            print(f'Using device {device_id}, named {device_name}')
            print('\n\n\n')
            # Record microphone for maximum of 10 seconds
            with sr.Microphone(device_index=device_id, sample_rate=int(rate), chunk_size=int(chunk)) as source:
                print(source.CHUNK)
                print(source.SAMPLE_RATE)
                print(source.SAMPLE_WIDTH)
                print('recording start')
                audio = recogniser.listen(source, timeout=5, phrase_time_limit=2)
            print('recording complete')

            # Save recording to file
            with open(f"mic-test_{device_id}.wav", "wb") as f:
                print('saving start')
                f.write(audio.get_wav_data())
            print('saving complete')

        except Exception as e:
            print(e)
            print('\n\n\n\n')



    def trigger1(self, msg):
        self.trigger(msg, self.pub1)

    def trigger2(self, msg):
        self.trigger(msg, self.pub2)

    def trigger3(self, msg):
        self.trigger(msg, self.pub3)

    def beep(self, data='i', speed=1000):
        cmd = 'espeak "%s" --stdout | sox -t wav - -r %s -t wav - remix 1 1 | aplay -D %s'
        os.system(cmd % (data, speed, self.out_device))

    def trigger(self, msg, pub):

        # obtain audio from the microphone
        print('\n\n\n')
        print('trigger recieved')
        with sr.Microphone(device_index=self.device_id, chunk_size=self.chunk_size) as source:
            print('\n'*10)
            print("Collecting Information:")
            print(f'Using device {self.device_id}, named {self.device_name}')

            try:
                #self.recogniser.adjust_for_ambient_noise(source)
                self.beep()
                print('begin listening...')
                audio = self.recogniser.listen(source, timeout=5, phrase_time_limit=5)
                print(dir(audio))
                print('end listening...')
            except sr.WaitTimeoutError:
                print('no audio collected')
                self.beep('uh', 10)
                self.pub0.publish(Empty())
                return
            print('audio recorded')

        # recognize speech using Google Speech Recognition
        try:
            # change API key from default using `...recognize_google(audio, key="?")`
            print('Beginning Transcription')
            text = self.recogniser.recognize_google(audio)
            self.say_failure = False

        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")

            # write audio to a WAV file
            with open("mic-result-failure.wav", "wb") as f:
                f.write(audio.get_wav_data())

            # print failure to speaker to help guide users
            if self.say_failure == False:
                self.say.publish(String(data='sorry, I could not understand that, but I will keep listening'))
            else:
                self.pub0.publish(Empty())
            self.say_failure = True

#            time.sleep(2)
#            self.pub0.publish(Empty())
            return

        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
#            time.sleep(2)
#            self.pub0.publish(Empty())
            return
        except Exception as e:
            print('other error')
            print(e)
            return

        print("Google Speech Recognition thinks you said: " + text)

        # write audio to a WAV file
        with open("mic-result-success.wav", "wb") as f:
            f.write(audio.get_wav_data())

        pub.publish(String(data=text))

        if text == 'end':
            self.sub = None

#        print('\n\n\n\n\n\n\n')
#        time.sleep(2)
#        self.pub0.publish(Empty())



def main(args=None):
    rclpy.init(args=args)
    in_device = os.getenv('ALSA_INPUT', 'hw:1,0')
    out_device = os.getenv('ALSA_OUTPUT', 'hw:2,0')

    AC = AudioCollector(in_device=in_device, out_device=out_device)
    rclpy.spin(AC)

    AC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
