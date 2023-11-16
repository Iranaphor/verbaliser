#!/usr/bin/env python3

# @desc: note, this example requires PyAudio because it uses the Microphone class
# @url: https://github.com/Uberi/speech_recognition/blob/master/examples/microphone_recognition.py

import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

import speech_recognition as sr


class AudioCollector(Node):

    def __init__(self, in_device, out_device):
        super().__init__('audio_collector')

        # Get specified microphone device
        self.out_device = out_device
        self.device = in_device
        self.device_list = []
        while True not in self.device_list:
            self.device_list = [in_device in m for m in sr.Microphone.list_microphone_names()]
        self.device_id = self.device_list.index(True)
        self.device_name = sr.Microphone.list_microphone_names()[self.device_id]

        print('\n'*50)
        print(f'Using device {self.device_id}, named {self.device_name}')

        # Construct recorder
        self.say_failure = False
        self.recogniser = sr.Recognizer()
        self.say = self.create_publisher(String, '/verbaliser/openai_reply', 10)

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

    def trigger1(self, msg):
        self.trigger(msg, self.pub1)

    def trigger2(self, msg):
        self.trigger(msg, self.pub2)

    def trigger3(self, msg):
        self.trigger(msg, self.pub3)

    def beep(self):
        os.system('espeak "u" --stdout | sox -t wav - -r 1000 -t wav - remix 1 1 | aplay -D %s'%self.out_device)

    def trigger(self, msg, pub):

        # obtain audio from the microphone
        with sr.Microphone(device_index=self.device_id) as source:
            print('\n'*10)
            print("Collecting Information:")
            print(f'Using device {self.device_id}, named {self.device_name}')

            try:
                #self.recogniser.adjust_for_ambient_noise(source)
                self.beep()
                audio = self.recogniser.listen(source, timeout=5, phrase_time_limit=5)
                print(audio)
            except sr.WaitTimeoutError:
                print('no audio collected')
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
            if self.say_failure == False:
                self.say.publish(String(data='sorry, I could not understand that, but I will keep listening'))
            else:
                self.pub0.publish(Empty())
            self.say_failure = True
            return
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
            return
        except Exception as e:
            print('other error')
            print(e)
            return

        print("Google Speech Recognition thinks you said: " + text)
        pub.publish(String(data=text))

        if text == 'end':
            self.sub = None

def main(args=None):
    rclpy.init(args=args)

    AC = AudioCollector(in_device='hw:3,0', out_device='hw:2,0')
    rclpy.spin(AC)

    AC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

