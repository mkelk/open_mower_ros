#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cherokey_msgs.msg import WheelState

def talker():
    
    pub = rospy.Publisher("test_msg_topic", String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    pub_wheels = rospy.Publisher("test_msg_cherokey_wheels", WheelState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(f"Sending {hello_str}")
        pub.publish(hello_str)
        pub_wheels.publish(10,1,100,-1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


