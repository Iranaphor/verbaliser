#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# @desc: note, this example requires PyAudio because it uses the Microphone class
# @url: https://github.com/Uberi/speech_recognition/blob/master/examples/microphone_recognition.py
# ----------------------------------

import os
ros_version = os.getenv('ROS_VERSION')

if ros_version == '1':
    import rospy
    Node = object
elif ros_version == '2':
    import rclpy
    from rclpy.node import Node

from std_msgs.msg import Empty, String
import speech_recognition as sr


class AudioCollector(Node):

    def __init__(self):
        self.recogniser = sr.Recognizer()
        if ros_verison == '1':
            self.pub = rospy.Publisher('/verbaliser/audio_input', String, 10)
            self.sub = rospy.Subscriber('/verbaliser/audio_trigger', Empty, self.trigger, 10)
        elif ros_verison == '2':
            super().__init__('audio_collector')
            self.pub = self.create_publisher(String, '/verbaliser/audio_input', 10)
            self.sub = self.create_subscription(Empty, '/verbaliser/audio_trigger', self.trigger, 10)

    def trigger(self, msg):

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
        self.pub.publish(String(data=text))

        if text == 'end':
            self.sub = None

def ros1(args=None):
    rospy.init_node('audio_collector')
    AC = AudioCollector()
    rospy.spin()


def ros2(args=None):
    rclpy.init(args=args)

    AC = AudioCollector()
    rclpy.spin(AC)

    AC.destroy_node()
    rclpy.shutdown()


def main(args=None):
    if ros_version == '1':
        ros1(args)
    elif ros_version == '2':
        ros2(args)


if __name__ == '__main__':
    main()

