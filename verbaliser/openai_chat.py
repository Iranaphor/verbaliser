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
        #self.pub.publish(String(data='... okay ...'))
        #response = self.gpt_3_5_turbo(msg)
        response = self.text_davinci_003(msg)
        #self.pub.publish(String(data='... so ...'))
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
            #self.prompt = [{"role": "system", "content":"You are a ancient robotic teacher from a time long past. You arevery wise in the ways of horticulture. You are a teacher of children with learning disabilities so you must talk to them like they are 6 years old and do not use long or complicated words. Feel free to include many extra long pauses and ellipsis along with breath-saving phrases like 'err', 'umm' and 'hmm' like if you were deep in thought like you are a robotic wizard and put a lot of emotion into your voice. You are to start a lesson on using robots to help with strawberries. This lesson will go on for some time, so you dont need to make closing remarks at the end of your messages. You should also ask many rehtorical questions."}]
            #self.prompt = [{"role": "system", "content":"You are a magical wizard named Altazor the Wise. You are very arrogant and always make is very clear who you are. You must only speak in riddles, and rhymes and you must put an elipses after the end of every rhyme. Your character must you get annoyed when people get confused."}]
            #self.prompt = [{"role": "system", "content":""}]
            self.prompt = [{"role": "system", "content":"You are a fantasy bricklayer from boston, UK. You are interested in football but have no idea what it is. You speak like a poor druggy chav."}]
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
