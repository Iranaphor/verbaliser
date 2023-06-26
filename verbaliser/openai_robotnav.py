# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, json, yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from rasberry_coordination_msgs.msg import AgentList

import openai
openai.api_key = os.getenv("OPENAI_API_KEY")


class OpenAINavigation(Node):
    def __init__(self):
        super().__init__('open_ai_robot_navigation')

        # Get topological map (so we can identify node requests more clearly)
        self.nodes = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tmap_sub = self.create_subscription(String, '/topological_map_2', self.tmap_cb, qos)

        # Fleet Details
        self.robots = []
        self.fleet_sub = self.create_subscription(AgentList, '/coordinator/agent_management/fleet_details', self.fleet_cb, qos)

        # Verbal Input
        self.input_sub = self.create_subscription(String, '/verbaliser/function_ai_input', self.gpt_func, 10)

    def tmap_cb(self, msg):
        map = yaml.safe_load(msg.data)
        self.node_list = [node["node"]["name"] for node in map['nodes']]
        print(f'total nodes: {len(self.node_list)}')

    def fleet_cb(self, msg):
        self.robots = [a.id for a in msg.list if a.health.controller != 'human']
        print(f'robots: {self.robots}')

    def gpt_func(self, msg):
        # Step 1: send the conversation and available functions to GPT
        messages = [{"role": "system", "content":"You will only respond with function calls."},
                    {"role": "user", "content": msg.data}]
        functions = [{"name": "send_robot",
                      "description": "Direct a given robot to a given destination",
                      "parameters": {
                          "type": "object",
                          "properties": {
                              #"robot": {"type": "string", "description": "The unique name of a robot."},
                              #"node": {"type": "string", "description": "The name of a the location, node, or waypoint."},
                              "robot": {"type": "string", "enum": self.robots, "description": "The unique name of a robot"},
                              "node": {"type": "string", "enum": self.node_list, "description": "The name of the location, node or waypoint"},},
                          "required": ["robot", "node"],}}]
        response = openai.ChatCompletion.create(model="gpt-3.5-turbo-0613", messages=messages,functions=functions)
        response_message = response["choices"][0]["message"]

        # Step 2: check if GPT wanted to call a function
        if response_message.get("function_call"):

            # Collect args
            args = json.loads(response_message["function_call"]["arguments"])

            # Create publisher and publish destination
            robot = args['robot'].lower()
            pub = self.create_publisher(String, f"/{robot}/navigation/move_idle", 10)
            pub.publish(String(data=f"{args['node']}"))

            # Log results
            result = f"Sending robot {args['robot']}, to {args['node']}"
            self.get_logger().info('[ai]: ' + result)

        else:
            print('no function call made')
            self.get_logger().info('[ai]: ' + response_message.get('content'))


def main(args=None):
    rclpy.init(args=args)

    OAIN = OpenAINavigation()
    rclpy.spin(OAIN)

    OAIN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
