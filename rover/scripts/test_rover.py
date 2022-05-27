#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.loginfo("test_rover is starting")

    # start node
    node = rospy.init_node("ck_joy_connect")
    
    pub_wheelie = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=10)

    msg = Twist()
    msg.linear.x = 0.5
    rospy.loginfo("Publishing /rover/cmd_vel move")
    pub_wheelie.publish(msg)   
    rospy.sleep(10.0)
    msg.linear.x = 0.0
    rospy.loginfo("Publishing /rover/cmd_vel stop")
    pub_wheelie.publish(msg)   
    
    rospy.loginfo("test_rover  is ending")

