#!/usr/bin/env python3

# ##################################################################
# Low-level control of Cherokey bot using rosserial
# ##################################################################

import rospy
from std_msgs.msg import String
# custom messages used by rosserial to communicate with Arduino on Cherokey
from cherokey_msgs.msg import WheelState

def talker():
    
    # publisher for messages to Cherokey
    pub = rospy.Publisher("test_msg_topic", String, queue_size=10)
    pub_wheels = rospy.Publisher("wheels_set_state", WheelState, queue_size=10)

    # start node
    rospy.init_node('talker', anonymous=True)
    
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(f"Sending {hello_str}")
        pub.publish(hello_str)
        pub_wheels.publish(100,1,100,1)
        rospy.sleep(1.0)
        pub_wheels.publish(100,-1,100,-1)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


