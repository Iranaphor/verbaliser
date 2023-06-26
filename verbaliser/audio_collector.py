#!/usr/bin/env python3

# @desc: note, this example requires PyAudio because it uses the Microphone class
# @url: https://github.com/Uberi/speech_recognition/blob/master/examples/microphone_recognition.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

import speech_recognition as sr


class AudioCollector(Node):

    def __init__(self):
        super().__init__('audio_collector')
        self.recogniser = sr.Recognizer()
        self.pub1 = self.create_publisher(String, '/verbaliser/smart_ai_input', 10)
        self.pub2 = self.create_publisher(String, '/verbaliser/basic_ai_input', 10)
        self.pub3 = self.create_publisher(String, '/verbaliser/function_ai_input', 10)
        self.sub1 = self.create_subscription(Empty, '/verbaliser/smart_ai_audio_trigger', self.trigger1, 10)
        self.sub2 = self.create_subscription(Empty, '/verbaliser/basic_ai_audio_trigger', self.trigger2, 10)
        self.sub3 = self.create_subscription(Empty, '/verbaliser/function_ai_audio_trigger', self.trigger3, 10)

    def trigger1(self, msg):
        self.trigger(msg, self.pub1)

    def trigger2(self, msg):
        self.trigger(msg, self.pub2)

    def trigger3(self, msg):
        self.trigger(msg, self.pub3)

    def trigger(self, msg, pub):

        # obtain audio from the microphone
        with sr.Microphone() as source:
            print('\n'*3)
            print("Collecting Information:")
            try:
                audio = self.recogniser.listen(source, timeout=5, phrase_time_limit=10)
            except sr.WaitTimeoutError:
                print('no audio collected')
                return

        # recognize speech using Google Speech Recognition
        try:
            # change API key from default using `...recognize_google(audio, key="?")`
            text = self.recogniser.recognize_google(audio)
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
            return
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
            return
        except:
            return

        print("Google Speech Recognition thinks you said: " + text)
        pub.publish(String(data=text))

        if text == 'end':
            self.sub = None

def main(args=None):
    rclpy.init(args=args)

    AC = AudioCollector()
    rclpy.spin(AC)

    AC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

