#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

pub_ck = None

def cmd_vel_callback(msg):   
    rospy.loginfo("In cmd_vel_callback")
    pub_ck.publish(msg)     


if __name__ == '__main__':
    rospy.loginfo("ck_joy_connect is starting")

    # start node
    node = rospy.init_node("ck_joy_connect")
    
    # start publisher for sending data over rosserial to Arduino on Cherokey
    pub_ck = rospy.Publisher("/cherokey/cmd_vel", Twist, queue_size=10)

    # create subscribers
    cmd_vel_subscription = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()

    rospy.loginfo("ck_joy_connect is ending")

