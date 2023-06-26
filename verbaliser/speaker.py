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


class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.sub = self.create_subscription(String, '/verbaliser/openai_reply', self.callback, 10)
        #self.pub = self.create_publisher(Empty, '/verbaliser/audio_trigger', 10)

    def callback(self, msg):
        self.get_logger().info(msg.data)
        sentance_list = msg.data.replace('"',',').replace('   ', '...').split('...')
        for sentance in sentance_list:
            os.system('espeak "%s"' % sentance)
            #os.system('~/tts_test.sh "%s"' % sentance)
            if sentance is not sentance_list[-1]:
                if random.randint(0, 1):
                    time.sleep(0.25)
                    os.system('espeak "%s"' % '...')
                    #os.system('~/tts_test.sh "..."')
                time.sleep(0.75)
        #if msg.data.startswith('...'):
        #    return
        #self.pub.publish(Empty())

def main(args=None):
    rclpy.init(args=args)

    SP = Speaker()
    rclpy.spin(SP)

    SP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
