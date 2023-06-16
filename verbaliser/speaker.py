#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
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
import time, random


class Speaker(Node):
    def __init__(self):
        if ros_version == '1':
            self.sub = rospy.Subscriber('/verbaliser/openai_reply', String, self.callback, 10)
            self.pub = rospy.Publisher('/verbaliser/audio_trigger', Empty, queue_size=10)
        elif ros_verison == '2':
            super().__init__('speaker')
            self.sub = self.create_subscription(String, '/verbaliser/openai_reply', self.callback, 10)
            self.pub = self.create_publisher(Empty, '/verbaliser/audio_trigger', 10)

    def callback(self, msg):
        if ros_version == '1':
            rospy.loginfo(msg.data)
        elif ros_version == '2':
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
        if msg.data.startswith('...'):
            return
        self.pub.publish(Empty())


def ros1(args=None):
    rospy.init_node('speaker')
    SP = Speaker()
    rospy.spin()


def ros2(args=None):
    rclpy.init(args=args)

    SP = Speaker()
    rclpy.spin(SP)

    SP.destroy_node()
    rclpy.shutdown()


def main(args=None):
    if ros_version == '1':
        ros1(args)
    elif ros_version == '2':
        ros2(args)


if __name__ == '__main__':
    main()

