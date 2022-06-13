#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cherokey_msgs.msg import WheelState

def talker():
    
    pub_wheels = rospy.Publisher("wheels_set_state", WheelState, queue_size=10)
    rospy.init_node('stopper', anonymous=True)
    
    
    while not rospy.is_shutdown():
        pub_wheels.publish(0,1,0,1)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


