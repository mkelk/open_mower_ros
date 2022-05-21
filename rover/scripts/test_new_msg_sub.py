#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def test_msg_topic_cb(msg):
    rospy.loginfo(f"got message {msg.data}")

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("test_msg_topic", String, test_msg_topic_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()