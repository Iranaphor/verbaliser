# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, time, random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty

# Types of espeak
espeak_standard = 'espeak "%s"'
espeak_docker = 'espeak "%s" --stdout | sox -t wav - -r 48000 -t wav - remix 1 1 | aplay -D hw:2,0'


class Speaker(Node):
    def __init__(self, use_docker_method=False):
        super().__init__('speaker')
        self.use_docker_method = use_docker_method
        self.sub = self.create_subscription(String, '/verbaliser/openai_reply', self.callback, 10)
        self.pub = self.create_publisher(Empty, '/verbaliser/audio_trigger', 10)

    def callback(self, msg):
        self.get_logger().info(msg.data)

        # Convert message to list of sentences
        sentence_list = msg.data.replace('"',',').replace('   ', '...').split('...')

        for sentence in sentence_list:

            # Output the speech to docker
            if self.use_docker_method:
                os.system(espeak_docker % sentence)
            else:
                os.system(espeak_standard % sentence)
        
            # Add a pause if more then one sentence
            if sentence is not sentence_list[-1]:
                if random.randint(0, 1):
                    time.sleep(0.25)
                    os.system('espeak "%s"' % '...')
                time.sleep(0.75)

        self.pub.publish(Empty())

def main(args=None):
    rclpy.init(args=args)

    SP = Speaker(use_docker_method=True)
    rclpy.spin(SP)

    SP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
