# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty


class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.sub = self.create_subscription(String, '~/input', self.callback, 10)
        self.pub = self.create_publisher(Empty, '~/trigger', 10)

    def callback(self, msg):
        self.get_logger().info(msg.data)
        os.system('espeak "%s"' % msg.data)
        self.pub.publish(Empty())

def main(args=None):
    rclpy.init(args=args)

    SP = Speaker()
    rclpy.spin(SP)

    SP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
