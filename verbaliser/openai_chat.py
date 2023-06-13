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
        self.sub = self.create_subscription(String, '/verbaliser/audio_input', self.cb, 10)
        self.pub = self.create_publisher(String, '/verbaliser/openai_reply', 10)

    def cb(self, msg):
        # Recieve the input text from the creator and publish a response from the selected ai
        self.get_logger().info('[ User ]: ' + msg.data)
        response = self.gpt_3_5_turbo(msg)
        self.get_logger().info('[ AI   ]: ' + response)
        self.pub.publish(String(data=response))

    def text_davinci_003(self, msg):
        # Update prompt, creating a new one if not existing
        if not hasattr(self, 'prompt'):
            self.prompt = "You: "
        self.prompt += msg.data + "\nFriend:"

        # Make the API Request to the davinci system
        reply = openai.Completion.create(
          model="text-davinci-003",
          prompt=self.prompt,
          temperature=0.5,
          max_tokens=60,
          top_p=1.0,
          frequency_penalty=0.5,
          presence_penalty=0.0,
          stop=["You:"]
        )
        response = reply.to_dict()['choices'][0].to_dict()['text'].replace('\n', ' ')[1:]

        # Update the log of the conversation and return the result
        self.prompt += response + "\nYou: "
        return response

    def gpt_3_5_turbo(self, msg):
        # Update prompt, creating a new one if not existing
        if not hasattr(self, 'prompt'):
            self.prompt = [{"role": "system", "content":"You are a snarky wisecracking salesman"}]
        self.prompt += [{"role": "user", "content": msg.data}]

        # Make the API Request to the davinci system
        reply = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=self.prompt)
        response = reply.to_dict()['choices'][0].to_dict()['message'].to_dict()['content'].replace('\n', ' ')

        # Update the log of the conversation and return the result
        self.prompt += [{"role": "assistant", "content": response}]
        return response


def main(args=None):
    rclpy.init(args=args)

    OAI = OpenAI()
    rclpy.spin(OAI)

    OAI.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
