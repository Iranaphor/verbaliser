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
from openai.error import InvalidRequestError


class OpenAI(Node):
    def __init__(self):
        super().__init__('open_ai')

        # Define prompt
        #prompt = "You are a ROS2 specialist who is very technically gifted. "
        #prompt += "When you recieve questions about programming, respond only with the single line of code solution to the problem, do not include a description of the solution."

        prompt = "Your name is Limo, you are a robot wizard built by AgileX. "
        prompt += "You are on a quest to find Rob the bald man; and to defeat Xumin, the aphid man. You are travelling with lazy-guy Callum."
        prompt += "You must always answer with only a few sentences unless asked for a story."
        prompt += "You must always stay on topic to the quest and quest-related activities."

        self.prompt = [ ['system', 'system', prompt] ]
        print('Current Prompt')
        print(self.prompt)
        self.sub0 = self.create_subscription(String, '/verbaliser/set_prompt', self.cb0, 10)

        # Activate communications
        self.sub1 = self.create_subscription(String, '/verbaliser/smart_ai_input', self.cb1, 10)
        self.sub2 = self.create_subscription(String, '/verbaliser/basic_ai_input', self.cb2, 10)
        self.pub = self.create_publisher(String, '/verbaliser/openai_reply', 10)

    def cb0(self, msg):
        self.prompt = [ ['system', 'system', msg.data] ]
        print('\n'*40+'Current Prompt')
        print(self.prompt)

    def cb1(self, msg):
        self.call_ai(msg.data, self.gpt, 'AI')
    def cb2(self, msg):
        self.call_ai(msg.data, self.davinci, 'Ai')


    def call_ai(self, data, ai, id):
        # Update stored prompt
        self.prompt += [["user", "You", data]]
        print('\n')
        self.get_logger().info('[ User]: ' + data)

        # Get appropriate response
        response = ai()

        # Update stored prompt
        self.prompt += [['assistant', 'Friend', response]]
        print('\n')
        self.get_logger().info(f'[{id}]: ' + response)

        # Publish Message
        self.pub.publish(String(data=response))


    def davinci(self):
        # Format the prompt
        chat = "\n".join([f"{p[1]}: {p[2]}" for p in self.prompt[1:]])+" Friend: "

        # Make the API Request to the davinci system
        reply = openai.Completion.create(
          model="text-davinci-003",
          prompt=chat,
          temperature=0.5,
          max_tokens=60,
          top_p=1.0,
          frequency_penalty=0.5,
          presence_penalty=0.0,
          stop=["You:"]
        )
        return reply.to_dict()['choices'][0].to_dict()['text'].replace('\n', ' ')


    def gpt(self):
        # Format the prompt
        chat = [{"role": "system", "content": self.prompt[0][2]}] + \
               [{"role": p[0], "content": p[2]} for p in self.prompt[1:]]

        # Make the API Request to GPT3.5
        try:
            reply = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=chat)
            full_reply = reply.to_dict()['choices'][0].to_dict()['message'].to_dict()['content'].replace('\n', ' ')

        except InvalidRequestError:
            print('hit error, sending individual messages now')

        return full_reply


def main(args=None):
    rclpy.init(args=args)

    OAI = OpenAI()
    rclpy.spin(OAI)

    OAI.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
