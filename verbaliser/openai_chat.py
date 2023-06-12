# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import openai
openai.api_key = os.getenv("OPENAI_API_KEY")


class OpenAI(Node):
    def __init__(self):
        super().__init__('open_ai')
        self.prompt = "You: "
        self.sub = self.create_subscription(String, '/verbaliser/audio_input', self.cb, 10)
        self.pub = self.create_publisher(String, '/verbaliser/openai_reply', 10)

    def cb(self, msg):
        self.get_logger().info('User: ' + msg.data)
        self.prompt += msg.data + "\nFriend:"

        response = openai.Completion.create(
          model="text-davinci-003",
          prompt=self.prompt,
          temperature=0.5,
          max_tokens=60,
          top_p=1.0,
          frequency_penalty=0.5,
          presence_penalty=0.0,
          stop=["You:"]
        )

        friend = response.to_dict()['choices'][0].to_dict()['text'].replace('\n', '')
        self.get_logger().info('AI: ' + friend)

        self.prompt += friend + "\nYou: "
        self.pub.publish(String(data=friend))


def main(args=None):
    rclpy.init(args=args)

    OAI = OpenAI()
    rclpy.spin(OAI)

    OAI.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
