# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from sensor_msgs.msg import Joy


class TriggerDetector(Node):
    def __init__(self):
        super().__init__('trigger_detector')
        self.prior = Joy()
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self.pub1 = self.create_publisher(Empty, '/verbaliser/smart_ai_audio_trigger', 10)
        self.pub2 = self.create_publisher(Empty, '/verbaliser/basic_ai_audio_trigger', 10)
        self.pub3 = self.create_publisher(Empty, '/verbaliser/function_ai_audio_trigger', 10)

    def cb(self, msg):
        if msg.buttons[2] and not self.prior.buttons[2]:
            self.get_logger().info('Trigger sent, (X) button pressed.')
            self.pub3.publish(Empty())
        elif msg.buttons[3] and not self.prior.buttons[3]:
            self.get_logger().info('Trigger sent, (Y) button pressed.')
            self.pub1.publish(Empty())
        elif msg.buttons[8] and not self.prior.buttons[8]:
            self.get_logger().info('Trigger sent, logo button pressed.')
            self.pub2.publish(Empty())
        self.prior = msg

def main(args=None):
    rclpy.init(args=args)

    TD = TriggerDetector()
    rclpy.spin(TD)

    TD.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
